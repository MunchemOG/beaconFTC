/*
 * Beacon native core: C API (shared interface I3 of the Beacon plan).
 *
 * Frames and units
 *   - Field frame: FTC field frame defined by the perimeter walls. Origin at the field centre,
 *     +z up, heading counter-clockwise positive. Metres and radians.
 *   - Odometry world: whatever frame the Pinpoint reports in (it starts wherever the Pinpoint
 *     was reset). The core keeps its own odometry-world -> field alignment, seeded by
 *     beacon_set_start_pose() and corrected by its own fixes.
 *   - Robot frame: +x forward, +y left, +z up, origin at the centre of rotation.
 *   - beacon_mount is T_robot_cam with the BeaconConfig convention:
 *     R_robot_cam = Rz(yaw) * Ry(pitch) * Rx(roll) * R_forward, all zeros = camera looking
 *     straight forward and level; pitch > 0 looks down, yaw > 0 looks left.
 *   - Timestamps: int64 nanoseconds on one monotonic clock (on the robot,
 *     SystemClock.elapsedRealtimeNanos()). Odometry and frame capture times must share it.
 *
 * Threading
 *   - beacon_push_odom, beacon_push_frame, beacon_latest_fix, beacon_status and
 *     beacon_debug_line may be called from any thread. None of them waits for vision work.
 *   - Frames are processed on the core's own worker thread. beacon_push_frame copies the
 *     image; if the worker still has an unprocessed frame it returns BEACON_DROPPED.
 *   - beacon_destroy joins the worker; call it once, after all other calls have returned.
 *
 * Non-finite inputs (NaN, inf) are rejected where they enter and never reach the core.
 *
 * MIT License. See beacon-native/LICENSE.
 */
#ifndef BEACON_BEACON_H
#define BEACON_BEACON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct beacon_ctx beacon_ctx;

/* Pinhole camera with OpenCV radial-tangential distortion (k1, k2, p1, p2, k3), pixels.
 * rs_readout_s: rolling-shutter readout time, first row to last row (0 = global shutter). */
typedef struct { int width, height; double fx, fy, cx, cy, dist[5]; double rs_readout_s; } beacon_camera;

/* T_robot_cam: camera optical frame in the robot frame (metres, radians). See above. */
typedef struct { double x, y, z, roll, pitch, yaw; } beacon_mount;      /* T_robot_cam */

/* Planar robot pose at time t_ns (metres, radians). */
typedef struct { int64_t t_ns; double x, y, heading; } beacon_pose2;    /* robot, metres/rad */

/* Absolute robot pose in the FIELD frame at the frame's capture time t_ns, with 1-sigma
 * uncertainty (sigma_xy: metres, largest axis; sigma_heading: radians). inliers: number of
 * image measurements that agreed. kind: bitmask of BEACON_FIX_* telling what constrained it. */
typedef struct { int64_t t_ns; double x, y, heading, sigma_xy, sigma_heading;
                 int inliers; int kind; } beacon_fix;                   /* FIELD frame */

enum { BEACON_OK = 0, BEACON_DROPPED = 1, BEACON_ERR = -1 };

/* kind bitmask. BEACON_FIX_NO_ODOM marks a fix made without any odometry (bench-only mode:
 * no beacon_push_odom calls; the core predicts with constant velocity). Never set on a robot. */
enum { BEACON_FIX_EDGES = 1, BEACON_FIX_POINTS = 2, BEACON_FIX_SEAMS = 4 };  /* kind bitmask */
enum { BEACON_FIX_NO_ODOM = 8 };

/* beacon_status() codes, equal to org.firstinspires.ftc.teamcode.beacon.BeaconStatus. */
enum { BEACON_STATUS_INVALID = 0,       /* NULL context */
       BEACON_STATUS_INITIALIZING = 1,  /* no start pose, no odometry yet, or no fix yet */
       BEACON_STATUS_TRACKING = 2,      /* a fix was accepted within params.lostAfterS */
       BEACON_STATUS_LOST = 3 };        /* vision degraded: no accepted fix for lostAfterS;
                                           pose should be pure Pinpoint */

/* Creates a context. field_json: the field model file CONTENTS (interface I1), not a path.
 * params_json: optional tuning overrides (see beacon-native/README.md), may be NULL.
 * On failure returns NULL and writes a message into err (if err != NULL, err_len > 0). */
beacon_ctx* beacon_create(const char* field_json, const beacon_camera*, const beacon_mount*,
                          const char* params_json /* nullable */, char* err, int err_len);

/* The team's typed start pose in the field frame, stamped with the time it applies to. */
void beacon_set_start_pose(beacon_ctx*, const beacon_pose2* field_pose);

/* One Pinpoint sample in the odometry world. Time-ordered; out-of-order samples are ignored. */
void beacon_push_odom(beacon_ctx*, const beacon_pose2* odom_world_pose);  /* any thread, O(1) */

/* One 8-bit grayscale frame. stride in bytes (>= width). Returns BEACON_OK when queued,
 * BEACON_DROPPED when the worker is busy, BEACON_ERR on bad arguments or size mismatch. */
int  beacon_push_frame(beacon_ctx*, int64_t capture_t_ns, const uint8_t* gray,
                       int width, int height, int stride);              /* copies; never blocks */

/* Copies the newest accepted fix into out. Returns 1 if it is new since the last call, else 0. */
int  beacon_latest_fix(beacon_ctx*, beacon_fix* out);   /* 1 = new fix since last call, 0 = none */

int  beacon_status(beacon_ctx*);                        /* BeaconStatus codes */

/* One human-readable telemetry line (always NUL-terminated when len > 0). */
void beacon_debug_line(beacon_ctx*, char* buf, int len);

void beacon_destroy(beacon_ctx*);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_BEACON_H */
