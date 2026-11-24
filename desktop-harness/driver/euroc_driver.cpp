// Beacon desktop harness: replay an EuRoC ASL sequence (cam0 + imu0 only) through OpenVINS.
//
// WHAT IS LOGGED (the Android port must log exactly the same thing, or its replay comparison means
// nothing):
//   trajectory.csv  "t_ns,x,y,z,qx,qy,qz,qw,status"  (project log format, 00-PROJECT.md)
//     - Pose of the IMU body frame {I} expressed in OpenVINS's global frame {G}: p_IinG and the
//       Hamilton quaternion q_ItoG (x,y,z,w), i.e. it rotates IMU-frame vectors into {G}.
//       OpenVINS stores JPL q_GtoI; its four numbers are identical to Hamilton q_ItoG, so they are
//       written unchanged (the same thing OpenVINS's own ROS visualizer publishes).
//     - {G} is gravity-aligned (+z up, opposite gravity), with origin and yaw fixed by the IMU pose
//       at initialisation. It is NOT the EuRoC Vicon/Leica frame; ate.py aligns with SE(3).
//     - On EuRoC the IMU frame is the dataset's body frame "B" (sensor.yaml T_BS of imu0 is
//       identity), which is what state_groundtruth_estimate0 reports, so no lever arm is needed.
//     - t_ns: camera timestamp of the frame just processed, shifted into the IMU clock by the
//       current camera-IMU time offset estimate (t_imu = t_cam + t_d), in integer nanoseconds.
//     - status: 1 initializing, 2 tracking, 3 lost/diverged (BeaconStatus.java). One row per
//       processed camera frame, including frames before initialisation (status 1, pose zeros).
//   features.csv    "t_ns,id,u,v,track_len"  raw (distorted) pixel coordinates in cam0 of every
//                   feature the KLT front end holds in this frame; track_len counts consecutive
//                   frames (1 = newly detected).
//   points.csv      "t_ns,kind,x,y,z"  3D points in {G}: kind=slam for landmarks currently in the
//                   filter state, kind=msckf for features triangulated and used in this frame's
//                   MSCKF update.
//   timing.csv      "t_ns,frame_ms,imu_ms,num_feats,zupt,status"  wall time spent inside
//                   VioManager::feed_measurement_camera for this frame (image already decoded), and
//                   inside feed_measurement_imu for the IMU samples fed since the previous frame.
//   run_info.txt    key=value: dataset path, config path, counts.
//
// Feeding order mirrors OpenVINS's own serial ROS runner: IMU samples are fed in timestamp order and
// a camera frame is processed as soon as an IMU sample newer than (t_cam + t_d) has been fed, so the
// propagator always has IMU data spanning the image time. Units: EuRoC gives nanoseconds; OpenVINS
// takes double seconds (t_ns * 1e-9), rad/s and m/s^2.
//
// Usage: euroc_driver <estimator_config.yaml> <sequence_dir containing mav0/> <out_dir>
//                     [--realtime] [--max-frames N] [--verbosity LEVEL]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/imgcodecs.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "track/TrackBase.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace {

// Status codes, identical to TeamCode/.../beacon/BeaconStatus.java.
const int STATUS_INITIALIZING = 1;
const int STATUS_TRACKING = 2;
const int STATUS_LOST = 3;

// Fewer tracked features than this in a frame after initialisation is reported as "lost".
const int MIN_FEATS_TRACKING = 10;
// A speed above this (m/s) means the filter has diverged (EuRoC peaks near 2.3 m/s).
const double MAX_SANE_SPEED = 20.0;

// Exposes the front end and the ZUPT decision, which VioManager keeps protected. Subclassing
// avoids patching OpenVINS for read-only access.
class BeaconVioManager : public ov_msckf::VioManager {
public:
  explicit BeaconVioManager(ov_msckf::VioManagerOptions &p) : ov_msckf::VioManager(p) {}
  std::shared_ptr<ov_core::TrackBase> tracker() { return trackFEATS; }
  bool did_zupt() const { return did_zupt_update; }
};

struct ImuRow {
  long long t_ns;
  double w[3];
  double a[3];
};

struct CamRow {
  long long t_ns;
  std::string file;
};

std::string trim(const std::string &s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  size_t e = s.find_last_not_of(" \t\r\n");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

bool read_imu_csv(const std::string &path, std::vector<ImuRow> &out) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> cols;
    while (std::getline(ss, tok, ',')) cols.push_back(trim(tok));
    if (cols.size() < 7) continue;
    ImuRow r;
    r.t_ns = std::stoll(cols[0]);
    for (int k = 0; k < 3; k++) {
      r.w[k] = std::stod(cols[1 + k]); // w_RS_S [rad/s]
      r.a[k] = std::stod(cols[4 + k]); // a_RS_S [m/s^2]
    }
    out.push_back(r);
  }
  return !out.empty();
}

bool read_cam_csv(const std::string &path, std::vector<CamRow> &out) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    size_t c = line.find(',');
    if (c == std::string::npos) continue;
    CamRow r;
    r.t_ns = std::stoll(trim(line.substr(0, c)));
    r.file = trim(line.substr(c + 1));
    out.push_back(r);
  }
  return !out.empty();
}

double ms_since(const std::chrono::steady_clock::time_point &t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void usage() {
  std::fprintf(stderr, "usage: euroc_driver <estimator_config.yaml> <sequence_dir> <out_dir> "
                       "[--realtime] [--max-frames N] [--verbosity ALL|DEBUG|INFO|WARNING|ERROR|SILENT]\n");
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    usage();
    return 2;
  }
  const std::string config_path = argv[1];
  const std::string seq_dir = argv[2];
  const std::string out_dir = argv[3];
  bool realtime = false;
  long max_frames = -1;
  std::string verbosity_override;
  for (int i = 4; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--realtime") {
      realtime = true;
    } else if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::atol(argv[++i]);
    } else if (a == "--verbosity" && i + 1 < argc) {
      verbosity_override = argv[++i];
    } else {
      usage();
      return 2;
    }
  }

  // ---- load data -------------------------------------------------------------------------
  const std::string mav0 = seq_dir + "/mav0";
  std::vector<ImuRow> imu;
  std::vector<CamRow> cams;
  if (!read_imu_csv(mav0 + "/imu0/data.csv", imu)) {
    std::fprintf(stderr, "cannot read %s/imu0/data.csv\n", mav0.c_str());
    return 1;
  }
  if (!read_cam_csv(mav0 + "/cam0/data.csv", cams)) {
    std::fprintf(stderr, "cannot read %s/cam0/data.csv\n", mav0.c_str());
    return 1;
  }
  std::printf("[driver] %zu IMU samples, %zu cam0 frames\n", imu.size(), cams.size());

  // ---- configure OpenVINS ----------------------------------------------------------------
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  std::string verbosity = "WARNING";
  parser->parse_config("verbosity", verbosity);
  if (!verbosity_override.empty()) verbosity = verbosity_override;
  ov_core::Printer::setPrintLevel(verbosity);

  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  // Serial, deterministic processing: initialisation runs inline on this thread (its cost lands in
  // the timing of the frame that triggers it, which is the honest number).
  params.use_multi_threading_subs = false;
  params.use_multi_threading_pubs = false;
  params.record_timing_information = false;
  if (!parser->successful()) {
    std::fprintf(stderr, "[driver] unable to parse all parameters in %s\n", config_path.c_str());
    return 1;
  }
  if (params.state_options.num_cameras != 1) {
    std::fprintf(stderr, "[driver] config must be monocular (max_cameras: 1), got %d\n", params.state_options.num_cameras);
    return 1;
  }
  auto sys = std::make_shared<BeaconVioManager>(params);

  // ---- outputs ---------------------------------------------------------------------------
  ::mkdir(out_dir.c_str(), 0755);
  std::FILE *f_traj = std::fopen((out_dir + "/trajectory.csv").c_str(), "w");
  std::FILE *f_feat = std::fopen((out_dir + "/features.csv").c_str(), "w");
  std::FILE *f_pts = std::fopen((out_dir + "/points.csv").c_str(), "w");
  std::FILE *f_time = std::fopen((out_dir + "/timing.csv").c_str(), "w");
  if (!f_traj || !f_feat || !f_pts || !f_time) {
    std::fprintf(stderr, "[driver] cannot open output files in %s\n", out_dir.c_str());
    return 1;
  }
  std::fprintf(f_traj, "t_ns,x,y,z,qx,qy,qz,qw,status\n");
  std::fprintf(f_feat, "t_ns,id,u,v,track_len\n");
  std::fprintf(f_pts, "t_ns,kind,x,y,z\n");
  std::fprintf(f_time, "t_ns,frame_ms,imu_ms,num_feats,zupt,status\n");

  // ---- replay ----------------------------------------------------------------------------
  const double min_frame_dt = 1.0 / params.track_frequency; // same frame-drop rule as OpenVINS's ROS node
  double last_frame_t = -1e300;
  std::unordered_map<size_t, int> track_len;      // feature id -> consecutive frames seen
  std::unordered_map<size_t, int> track_len_next; // reused per frame
  size_t ci = 0;
  long frames_done = 0, frames_tracking = 0, frames_lost = 0, frames_zupt = 0;
  double imu_ms_acc = 0.0;
  bool was_initialized = false;
  const auto wall0 = std::chrono::steady_clock::now();
  const long long data_t0 = cams.front().t_ns;

  for (size_t ii = 0; ii < imu.size() && ci < cams.size(); ii++) {
    const ImuRow &r = imu[ii];
    ov_core::ImuData m;
    m.timestamp = (double)r.t_ns * 1e-9;
    m.wm << r.w[0], r.w[1], r.w[2];
    m.am << r.a[0], r.a[1], r.a[2];
    auto ti = std::chrono::steady_clock::now();
    sys->feed_measurement_imu(m);
    imu_ms_acc += ms_since(ti);

    // Process every frame whose time (in the IMU clock) is now covered by IMU data.
    const double t_d = sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
    const double imu_t_in_cam = m.timestamp - t_d;
    while (ci < cams.size() && (double)cams[ci].t_ns * 1e-9 < imu_t_in_cam) {
      const CamRow &c = cams[ci++];
      const double tc = (double)c.t_ns * 1e-9;
      if (tc < last_frame_t + min_frame_dt) continue; // dropped by track_frequency
      last_frame_t = tc;

      if (realtime) {
        // Sleep until wall time catches up with data time.
        double due_ms = (double)(c.t_ns - data_t0) * 1e-6;
        double ahead = due_ms - ms_since(wall0);
        if (ahead > 0) std::this_thread::sleep_for(std::chrono::microseconds((long long)(ahead * 1000)));
      }

      ov_core::CameraData cm;
      cm.timestamp = tc;
      cm.sensor_ids.push_back(0);
      cv::Mat img = cv::imread(mav0 + "/cam0/data/" + c.file, cv::IMREAD_GRAYSCALE);
      if (img.empty()) {
        std::fprintf(stderr, "[driver] cannot read image %s, skipping\n", c.file.c_str());
        continue;
      }
      cm.images.push_back(img);
      if (params.use_mask) {
        cm.masks.push_back(params.masks.at(0));
      } else {
        cm.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
      }

      auto tf = std::chrono::steady_clock::now();
      sys->feed_measurement_camera(cm);
      const double frame_ms = ms_since(tf);

      // ---- pose + status ----
      std::shared_ptr<ov_msckf::State> state = sys->get_state();
      const bool inited = sys->initialized();
      Eigen::Vector4d q = state->_imu->quat(); // JPL q_GtoI == Hamilton q_ItoG, order x,y,z,w
      Eigen::Vector3d p = state->_imu->pos();  // p_IinG
      Eigen::Vector3d v = state->_imu->vel();
      const double td_now = state->_calib_dt_CAMtoIMU->value()(0);
      const long long t_out = c.t_ns + (long long)std::llround(td_now * 1e9);

      // ---- features: every KLT track in this frame ----
      auto tracker = sys->tracker();
      auto obs = tracker->get_last_obs();
      auto ids = tracker->get_last_ids();
      int num_feats = 0;
      track_len_next.clear();
      if (obs.count(0) && ids.count(0)) {
        const auto &kp = obs.at(0);
        const auto &id = ids.at(0);
        for (size_t k = 0; k < kp.size() && k < id.size(); k++) {
          auto it = track_len.find(id[k]);
          int len = (it == track_len.end()) ? 1 : it->second + 1;
          track_len_next[id[k]] = len;
          std::fprintf(f_feat, "%lld,%zu,%.2f,%.2f,%d\n", t_out, id[k], kp[k].pt.x, kp[k].pt.y, len);
        }
        num_feats = (int)kp.size();
      }
      track_len.swap(track_len_next);

      int status = STATUS_INITIALIZING;
      if (inited) {
        const bool finite = p.allFinite() && q.allFinite() && v.allFinite();
        const bool diverged = !finite || v.norm() > MAX_SANE_SPEED;
        status = (diverged || num_feats < MIN_FEATS_TRACKING) ? STATUS_LOST : STATUS_TRACKING;
      }
      if (inited && !was_initialized) {
        std::printf("[driver] initialized at frame %ld (t=%.3f s into the sequence)\n", frames_done,
                    (double)(c.t_ns - data_t0) * 1e-9);
        was_initialized = true;
      }
      if (!inited) {
        p.setZero();
        q << 0, 0, 0, 1;
      }
      std::fprintf(f_traj, "%lld,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%d\n", t_out, p(0), p(1), p(2), q(0), q(1), q(2), q(3), status);

      // ---- 3D points ----
      // On a zero-velocity frame OpenVINS returns before any feature update, so the MSCKF list
      // still holds the previous frame's points; skip it then to avoid logging duplicates.
      const bool zupt = inited && sys->did_zupt();
      if (inited) {
        for (const auto &pt : sys->get_features_SLAM())
          std::fprintf(f_pts, "%lld,slam,%.4f,%.4f,%.4f\n", t_out, pt(0), pt(1), pt(2));
        if (!zupt) {
          for (const auto &pt : sys->get_good_features_MSCKF())
            if (pt.allFinite()) std::fprintf(f_pts, "%lld,msckf,%.4f,%.4f,%.4f\n", t_out, pt(0), pt(1), pt(2));
        }
      }

      std::fprintf(f_time, "%lld,%.4f,%.4f,%d,%d,%d\n", t_out, frame_ms, imu_ms_acc, num_feats, zupt ? 1 : 0, status);
      imu_ms_acc = 0.0;

      frames_done++;
      if (status == STATUS_TRACKING) frames_tracking++;
      if (status == STATUS_LOST) frames_lost++;
      if (zupt) frames_zupt++;
      if (frames_done % 500 == 0) {
        std::printf("[driver] frame %ld/%zu  p=(%.2f, %.2f, %.2f)  feats=%d  %.2f ms\n", frames_done, cams.size(), p(0), p(1), p(2),
                    num_feats, frame_ms);
        std::fflush(stdout);
      }
      if (max_frames > 0 && frames_done >= max_frames) break;
    }
    if (max_frames > 0 && frames_done >= max_frames) break;
  }

  std::fclose(f_traj);
  std::fclose(f_feat);
  std::fclose(f_pts);
  std::fclose(f_time);

  std::FILE *f_info = std::fopen((out_dir + "/run_info.txt").c_str(), "w");
  if (f_info) {
    std::fprintf(f_info, "sequence_dir=%s\nconfig=%s\nframes=%ld\nframes_tracking=%ld\nframes_lost=%ld\nframes_zupt=%ld\nwall_s=%.3f\n",
                 seq_dir.c_str(), config_path.c_str(), frames_done, frames_tracking, frames_lost, frames_zupt, ms_since(wall0) * 1e-3);
    std::fclose(f_info);
  }
  std::printf("[driver] done: %ld frames (%ld tracking, %ld lost, %ld zupt) in %.1f s -> %s\n", frames_done, frames_tracking, frames_lost,
              frames_zupt, ms_since(wall0) * 1e-3, out_dir.c_str());
  return frames_tracking > 0 ? 0 : 3;
}
