package org.firstinspires.ftc.teamcode.beacon.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;

import java.lang.reflect.Field;
import java.net.InetAddress;

/**
 * Glue between the FTC SDK's {@link Limelight3A} driver and {@link LimelightFrameSource}.
 * Robot-only (uses SDK classes).
 *
 * <p>The SDK driver is used for two things only, both optional: (1) the Limelight address the
 * team set in the Robot Controller configuration, to build the stream URL; (2) the latency
 * the Limelight reports with each result ({@code cl} + {@code tl}), as a fallback latency
 * estimate. The SDK polls {@code /results} on port 5807 at 100 Hz on its own thread once
 * {@code Limelight3A.start()} has been called; reading the cached result here is a volatile
 * read. The frame source works without any Limelight in the configuration.
 */
public final class LimelightSdk {
    /** Results older than this are ignored for the latency estimate (ms). */
    static final long MAX_RESULT_AGE_MS = 250;

    private LimelightSdk() {}

    /**
     * The Limelight address from the RC configuration, or null. The SDK keeps it in a private
     * field ({@code Limelight3A.inetAddress}, Hardware 12.0.0); if that cannot be read, the
     * SDK's default derivation from the connection info is used instead.
     */
    public static String configuredIp(Limelight3A ll) {
        if (ll == null) {
            return null;
        }
        try {
            Field f = Limelight3A.class.getDeclaredField("inetAddress");
            f.setAccessible(true);
            Object v = f.get(ll);
            if (v instanceof InetAddress) {
                return ((InetAddress) v).getHostAddress();
            }
        } catch (Exception e) {
            // fall through: field renamed in a later SDK
        } catch (LinkageError e) {
            // fall through
        }
        try {
            return StreamCandidates.limelightIpFromConnectionInfo(ll.getConnectionInfo());
        } catch (RuntimeException e) {
            return null;
        }
    }

    /** cl + tl of the newest fresh result, NaN if there is none. */
    public static LimelightFrameSource.LatencyHint latencyHint(final Limelight3A ll) {
        if (ll == null) {
            return null;
        }
        return new LimelightFrameSource.LatencyHint() {
            @Override
            public double limelightLatencyMs() {
                LLResult r = ll.getLatestResult();
                if (r == null || r.getStaleness() > MAX_RESULT_AGE_MS) {
                    return Double.NaN;
                }
                double ms = r.getCaptureLatency() + r.getTargetingLatency();
                return ms > 0 ? ms : Double.NaN;   // an empty placeholder result has 0 + 0
            }
        };
    }

    /** The production frame source: candidate URLs, BitmapFactory decoding, Hub clock. */
    public static LimelightFrameSource createFrameSource(BeaconConfig cfg, Limelight3A llOrNull) {
        return new LimelightFrameSource(
                StreamCandidates.build(cfg.stream.streamUrl, configuredIp(llOrNull)),
                new LimelightFrameSource.HttpOpener(), new AndroidJpegDecoder(),
                latencyHint(llOrNull), LimelightFrameSource.hubClock(), cfg.stream);
    }
}
