// JNI shim between org.firstinspires.ftc.teamcode.beacon.vision.NativeBeacon (Java) and the
// Beacon native core's C API (beacon-native/include/beacon/beacon.h, plan interface I3).
// One Java native method per C function; no logic of its own beyond argument checks.
//
// Rules kept here:
//  - Nothing throws across the JNI boundary: every entry point catches all C++ exceptions.
//  - Strings handed to NewStringUTF are reduced to 7-bit ASCII first (NewStringUTF needs
//    modified UTF-8 and aborts the app on invalid bytes under CheckJNI).
//  - Frame pixels are read with GetPrimitiveArrayCritical (no copy); beacon_push_frame copies
//    them and never blocks, so the critical section is short.

#include <jni.h>

#include <cstdint>
#include <cstring>

#include "beacon/beacon.h"

namespace {

beacon_ctx* ctx(jlong h) { return reinterpret_cast<beacon_ctx*>(static_cast<intptr_t>(h)); }

jstring ascii(JNIEnv* env, const char* s) {
  char buf[1024];
  size_t n = 0;
  for (; s != nullptr && s[n] != '\0' && n < sizeof(buf) - 1; ++n) {
    unsigned char c = static_cast<unsigned char>(s[n]);
    buf[n] = (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '?';
  }
  buf[n] = '\0';
  return env->NewStringUTF(buf);
}

void setErr(JNIEnv* env, jobjectArray errOut, const char* msg) {
  if (errOut != nullptr && env->GetArrayLength(errOut) > 0) {
    jstring js = ascii(env, msg);
    if (js != nullptr) {
      env->SetObjectArrayElement(errOut, 0, js);
      env->DeleteLocalRef(js);
    }
  }
}

}  // namespace

#define BEACON_JNI(ret, name) \
  extern "C" JNIEXPORT ret JNICALL Java_org_firstinspires_ftc_teamcode_beacon_vision_NativeBeacon_##name

BEACON_JNI(jlong, nativeCreate)(JNIEnv* env, jclass, jstring field, jint width, jint height,
                                jdouble fx, jdouble fy, jdouble cx, jdouble cy,
                                jdoubleArray dist, jdouble rs, jdouble mx, jdouble my,
                                jdouble mz, jdouble mroll, jdouble mpitch, jdouble myaw,
                                jstring params, jobjectArray errOut) {
  if (field == nullptr || dist == nullptr || env->GetArrayLength(dist) != 5) {
    setErr(env, errOut, "beacon_jni: field model and 5 distortion terms are required");
    return 0;
  }
  const char* f = env->GetStringUTFChars(field, nullptr);
  if (f == nullptr) {
    return 0;  // OutOfMemoryError pending in Java
  }
  const char* p = params != nullptr ? env->GetStringUTFChars(params, nullptr) : nullptr;

  beacon_camera cam;
  std::memset(&cam, 0, sizeof(cam));
  cam.width = width;
  cam.height = height;
  cam.fx = fx;
  cam.fy = fy;
  cam.cx = cx;
  cam.cy = cy;
  env->GetDoubleArrayRegion(dist, 0, 5, cam.dist);
  cam.rs_readout_s = rs;
  beacon_mount mount = {mx, my, mz, mroll, mpitch, myaw};

  char err[512];
  err[0] = '\0';
  beacon_ctx* c = nullptr;
  try {
    c = beacon_create(f, &cam, &mount, p, err, static_cast<int>(sizeof(err)));
  } catch (...) {
    c = nullptr;
    std::strncpy(err, "beacon_create threw a C++ exception", sizeof(err) - 1);
    err[sizeof(err) - 1] = '\0';
  }
  env->ReleaseStringUTFChars(field, f);
  if (p != nullptr) {
    env->ReleaseStringUTFChars(params, p);
  }
  if (c == nullptr) {
    setErr(env, errOut, err[0] != '\0' ? err : "beacon_create failed");
    return 0;
  }
  return static_cast<jlong>(reinterpret_cast<intptr_t>(c));
}

BEACON_JNI(void, nativeSetStartPose)(JNIEnv*, jclass, jlong h, jlong t, jdouble x, jdouble y,
                                     jdouble heading) {
  beacon_pose2 p = {static_cast<int64_t>(t), x, y, heading};
  try {
    beacon_set_start_pose(ctx(h), &p);
  } catch (...) {
  }
}

BEACON_JNI(void, nativePushOdom)(JNIEnv*, jclass, jlong h, jlong t, jdouble x, jdouble y,
                                 jdouble heading) {
  beacon_pose2 p = {static_cast<int64_t>(t), x, y, heading};
  try {
    beacon_push_odom(ctx(h), &p);
  } catch (...) {
  }
}

BEACON_JNI(jint, nativePushFrame)(JNIEnv* env, jclass, jlong h, jlong t, jbyteArray gray,
                                  jint width, jint height, jint stride) {
  if (gray == nullptr || width <= 0 || height <= 0 || stride < width) {
    return BEACON_ERR;
  }
  jsize len = env->GetArrayLength(gray);
  if (static_cast<int64_t>(stride) * (height - 1) + width > len) {
    return BEACON_ERR;
  }
  void* px = env->GetPrimitiveArrayCritical(gray, nullptr);
  if (px == nullptr) {
    return BEACON_ERR;
  }
  int r = BEACON_ERR;
  try {
    r = beacon_push_frame(ctx(h), static_cast<int64_t>(t), static_cast<const uint8_t*>(px),
                          width, height, stride);
  } catch (...) {
    r = BEACON_ERR;
  }
  env->ReleasePrimitiveArrayCritical(gray, px, JNI_ABORT);  // read only: nothing to copy back
  return r;
}

BEACON_JNI(jint, nativeLatestFix)(JNIEnv* env, jclass, jlong h, jlongArray tOut,
                                  jdoubleArray vOut) {
  if (tOut == nullptr || vOut == nullptr || env->GetArrayLength(tOut) < 1 ||
      env->GetArrayLength(vOut) < 7) {
    return 0;
  }
  beacon_fix fx;
  std::memset(&fx, 0, sizeof(fx));
  int r = 0;
  try {
    r = beacon_latest_fix(ctx(h), &fx);
  } catch (...) {
    r = 0;
  }
  if (r != 1) {
    return 0;
  }
  jlong t = static_cast<jlong>(fx.t_ns);
  jdouble v[7] = {fx.x, fx.y, fx.heading, fx.sigma_xy, fx.sigma_heading,
                  static_cast<jdouble>(fx.inliers), static_cast<jdouble>(fx.kind)};
  env->SetLongArrayRegion(tOut, 0, 1, &t);
  env->SetDoubleArrayRegion(vOut, 0, 7, v);
  return 1;
}

BEACON_JNI(jint, nativeStatus)(JNIEnv*, jclass, jlong h) {
  try {
    return beacon_status(ctx(h));
  } catch (...) {
    return BEACON_STATUS_INVALID;
  }
}

BEACON_JNI(jstring, nativeDebugLine)(JNIEnv* env, jclass, jlong h) {
  char buf[512];
  buf[0] = '\0';
  try {
    beacon_debug_line(ctx(h), buf, static_cast<int>(sizeof(buf)));
  } catch (...) {
    std::strncpy(buf, "beacon_debug_line threw", sizeof(buf) - 1);
  }
  buf[sizeof(buf) - 1] = '\0';
  return ascii(env, buf);
}

BEACON_JNI(void, nativeDestroy)(JNIEnv*, jclass, jlong h) {
  try {
    beacon_destroy(ctx(h));
  } catch (...) {
  }
}
