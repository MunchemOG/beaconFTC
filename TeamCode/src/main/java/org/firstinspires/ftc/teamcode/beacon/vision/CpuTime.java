package org.firstinspires.ftc.teamcode.beacon.vision;

/**
 * CPU-time readings for telemetry. On the robot these come from Android; on a desktop JVM
 * (unit tests, where android.* methods are stubs that throw) they return -1.
 */
public final class CpuTime {
    private static volatile boolean threadOk = true;
    private static volatile boolean processOk = true;

    private CpuTime() {}

    /** CPU time used by the calling thread (ns), or -1 if unavailable. */
    public static long threadNs() {
        if (!threadOk) {
            return -1;
        }
        try {
            return android.os.Debug.threadCpuTimeNanos();
        } catch (RuntimeException e) {
            threadOk = false;
        } catch (LinkageError e) {
            threadOk = false;
        }
        return -1;
    }

    /** CPU time used by the whole app process, all threads (ms), or -1 if unavailable. */
    public static long processMs() {
        if (!processOk) {
            return -1;
        }
        try {
            return android.os.Process.getElapsedCpuTime();
        } catch (RuntimeException e) {
            processOk = false;
        } catch (LinkageError e) {
            processOk = false;
        }
        return -1;
    }
}
