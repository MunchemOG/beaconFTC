package org.firstinspires.ftc.teamcode.beacon.vision;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;

/**
 * {@link BeaconCore} over JNI: TeamCode/src/main/cpp/beacon_jni.cpp, which calls libbeacon.so
 * (beacon-native). Both libraries are built by Gradle for arm64-v8a only.
 *
 * <p>Loading never throws. If {@code libbeacon_jni.so} is missing (desktop tests, an APK built
 * without it, or a Control Hub whose Android runs 32-bit only), {@link #isLoaded()} is false,
 * {@link #loadError()} says why, and every call on an instance is a harmless no-op that
 * reports nothing; Beacon then runs on the Pinpoint alone.
 */
public final class NativeBeacon implements BeaconCore {

    private static final boolean LOADED;
    private static final String LOAD_ERROR;

    static {
        boolean ok = false;
        String err = null;
        try {
            System.loadLibrary("beacon_jni");   // pulls in libbeacon.so (DT_NEEDED)
            ok = true;
        } catch (UnsatisfiedLinkError e) {
            err = "UnsatisfiedLinkError: " + e.getMessage();
        } catch (SecurityException e) {
            err = "SecurityException: " + e.getMessage();
        } catch (LinkageError e) {
            err = e.toString();
        }
        LOADED = ok;
        LOAD_ERROR = err;
    }

    /** True if the native libraries loaded in this process. */
    public static boolean isLoaded() {
        return LOADED;
    }

    /** Why loading failed, or null. */
    public static String loadError() {
        return LOAD_ERROR;
    }

    private long handle;      // beacon_ctx*, 0 = none

    @Override
    public synchronized boolean create(String fieldJson, Camera c, BeaconConfig.CameraMount m,
                                       String paramsJson, String[] err) {
        if (!LOADED) {
            err[0] = "native library not loaded: " + LOAD_ERROR;
            return false;
        }
        if (handle != 0) {
            err[0] = "already created";
            return false;
        }
        String[] e = new String[1];
        long h = nativeCreate(fieldJson, c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.dist,
                c.rollingShutterS, m.x, m.y, m.z, m.roll, m.pitch, m.yaw, paramsJson, e);
        if (h == 0) {
            err[0] = e[0] == null ? "beacon_create failed" : e[0];
            return false;
        }
        handle = h;
        return true;
    }

    @Override
    public void setStartPose(long tNs, double x, double y, double heading) {
        long h = handle;
        if (h != 0) {
            nativeSetStartPose(h, tNs, x, y, heading);
        }
    }

    @Override
    public void resetOdomWorld(long tNs) {
        long h = handle;
        if (h != 0) {
            nativeResetOdomWorld(h, tNs);
        }
    }

    @Override
    public void pushOdom(long tNs, double x, double y, double heading) {
        long h = handle;
        if (h != 0) {
            nativePushOdom(h, tNs, x, y, heading);
        }
    }

    @Override
    public int pushFrame(long captureTNs, byte[] gray, int width, int height, int stride) {
        long h = handle;
        if (h == 0) {
            return ERR;
        }
        return nativePushFrame(h, captureTNs, gray, width, height, stride);
    }

    @Override
    public boolean latestFix(long[] t, double[] v) {
        long h = handle;
        return h != 0 && nativeLatestFix(h, t, v) == 1;
    }

    @Override
    public int status() {
        long h = handle;
        return h == 0 ? 0 : nativeStatus(h);
    }

    @Override
    public String debugLine() {
        long h = handle;
        return h == 0 ? "no core" : nativeDebugLine(h);
    }

    @Override
    public synchronized void destroy() {
        long h = handle;
        handle = 0;
        if (h != 0) {
            nativeDestroy(h);
        }
    }

    // ------------------------------------------------------------ JNI (beacon_jni.cpp)

    private static native long nativeCreate(String fieldJson, int width, int height, double fx,
                                            double fy, double cx, double cy, double[] dist,
                                            double rollingShutterS, double mx, double my,
                                            double mz, double mroll, double mpitch, double myaw,
                                            String paramsJson, String[] errOut);

    private static native void nativeSetStartPose(long h, long tNs, double x, double y,
                                                  double heading);

    private static native void nativePushOdom(long h, long tNs, double x, double y,
                                              double heading);

    private static native void nativeResetOdomWorld(long h, long tNs);

    private static native int nativePushFrame(long h, long tNs, byte[] gray, int width,
                                              int height, int stride);

    private static native int nativeLatestFix(long h, long[] tOut, double[] vOut);

    private static native int nativeStatus(long h);

    private static native String nativeDebugLine(long h);

    private static native void nativeDestroy(long h);
}
