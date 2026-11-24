package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Pinpoint source + localizer, without hardware. */
public class OdometryTierTest {

    /** Public so the parked Limelight tests (another package) can drive a Pinpoint too. */
    public static final class FakeReader implements PinpointPoseSource.Reader {
        public String status = "READY";
        public double x, y, h;
        public boolean throwNext;
        public int updates, reconfigs;

        public void update() {
            updates++;
            if (throwNext) {
                throwNext = false;
                throw new RuntimeException("I2C NACK");
            }
        }
        public String statusName() { return status; }
        public double xMetres() { return x; }
        public double yMetres() { return y; }
        public double headingRadians() { return h; }
        public double loopHz() { return 1500; }
        public void reconfigure() { reconfigs++; }
    }

    /** Advances 1 ms per reading so consecutive stamps are realistic and increasing. */
    static final class StepClock implements PinpointPoseSource.TimeSource {
        long t = 1000000000L;
        public long nowNs() {
            t += 1000000L;
            return t;
        }
    }

    static final class FakeClock implements BeaconLocalizer.Clock {
        long now = 1;
        public long nanoTime() { return now; }
    }

    private static BeaconConfig odo() throws BeaconException {
        return BeaconConfig.parse("{\"tier\":\"odometry\"}");
    }

    private static PinpointPoseSource started(PinpointPoseSource s) throws BeaconException {
        s.start(null, null);
        return s;
    }

    @Test
    public void deviceStatusNamesFromTheSdkAreClassified() {
        assertEquals(PinpointPoseSource.K_READY, PinpointPoseSource.classify("READY"));
        assertEquals(PinpointPoseSource.K_WARMING, PinpointPoseSource.classify("NOT_READY"));
        assertEquals(PinpointPoseSource.K_WARMING, PinpointPoseSource.classify("CALIBRATING"));
        assertEquals(PinpointPoseSource.K_BAD_READ,
                PinpointPoseSource.classify("FAULT_BAD_READ"));
        assertEquals(PinpointPoseSource.K_FAULT,
                PinpointPoseSource.classify("FAULT_NO_PODS_DETECTED"));
        assertEquals(PinpointPoseSource.K_FAULT,
                PinpointPoseSource.classify("FAULT_X_POD_NOT_DETECTED"));
        assertEquals(PinpointPoseSource.K_FAULT, PinpointPoseSource.classify("FAULT_IMU_RUNAWAY"));
        assertEquals(PinpointPoseSource.K_FAULT, PinpointPoseSource.classify(null));
    }

    @Test
    public void pinpointPoseRoundTripsAndCameraMountHasNoEffect() throws Exception {
        // A wild camera mount must not change anything for a robot-pose source.
        BeaconConfig c = BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":{\"x\":0.2,"
                + "\"y\":-0.1,\"z\":0.3,\"roll\":0.4,\"pitch\":0.5,\"yaw\":0.6}}");
        FakeReader r = new FakeReader();
        StepClock sc = new StepClock();
        PinpointPoseSource src = new PinpointPoseSource(r, sc);
        assertTrue(src.reportsRobotPose());
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.start(null);
        loc.update();
        loc.setPose(0, 0, 0);
        sc.t += 1000000000L;                  // a second later (a 1 m step needs time)
        r.x = 1.0;
        r.y = -0.5;
        r.h = 1.2;
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(1.0, p.x, 1e-9);
        assertEquals(-0.5, p.y, 1e-9);
        assertEquals(1.2, p.heading, 1e-9);
    }

    @Test
    public void exactlyOneDeviceReadPerLocalizerUpdate() throws Exception {
        FakeReader r = new FakeReader();
        PinpointPoseSource src = new PinpointPoseSource(r, new StepClock());
        BeaconLocalizer loc = new BeaconLocalizer(src, odo());
        assertEquals(0, r.updates);
        loc.update();                 // not started: no I2C traffic
        assertEquals(0, r.updates);
        loc.start(null);
        for (int i = 0; i < 25; i++) {
            loc.update();
            src.latest();             // reading the sample must not touch the device
            src.isTracking();
        }
        assertEquals(25, r.updates);
    }

    @Test
    public void calibratingIsInitializingAndTheStartPoseWaitsForReady() throws Exception {
        FakeReader r = new FakeReader();
        r.status = "CALIBRATING";
        StepClock sc = new StepClock();
        BeaconLocalizer loc = new BeaconLocalizer(new PinpointPoseSource(r, sc), odo());
        loc.start(null);
        loc.setPose(1.0, 2.0, Math.PI / 2);
        for (int i = 0; i < 5; i++) {
            assertEquals(BeaconStatus.INITIALIZING, loc.update().status);
        }
        r.status = "READY";
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(1.0, p.x, 1e-12);
        assertEquals(2.0, p.y, 1e-12);
        sc.t += 1000000000L;
        r.x = 0.5;                            // drives 0.5 m forward in its own frame
        p = loc.update();
        assertEquals(1.0, p.x, 1e-9);
        assertEquals(2.5, p.y, 1e-9);
        assertEquals(Math.PI / 2, p.heading, 1e-9);
    }

    @Test
    public void startPressedBeforeThePinpointIsReadyExpiresTheStartPose() throws Exception {
        FakeReader r = new FakeReader();
        r.status = "CALIBRATING";
        BeaconLocalizer loc = new BeaconLocalizer(new PinpointPoseSource(r, new StepClock()),
                odo());
        loc.start(null);
        loc.setPose(1.0, 2.0, 0);
        loc.update();
        loc.onStart();                        // START while still calibrating
        r.status = "READY";
        Pose2dStamped p = loc.update();
        assertEquals("start pose may no longer be true", BeaconStatus.INITIALIZING, p.status);
        assertTrue(loc.isStartPoseExpired());
        assertTrue(loc.debugLine().contains("START POSE EXPIRED"));
    }

    @Test
    public void aBadReadPublishesNothingAndOnlyATimeoutMakesItLost() throws Exception {
        FakeReader r = new FakeReader();
        FakeClock clk = new FakeClock();
        PinpointPoseSource src = new PinpointPoseSource(r, new StepClock());
        BeaconLocalizer loc = new BeaconLocalizer(src, odo(), null, null, clk);
        loc.start(null);
        loc.update();
        loc.setPose(0, 0, 0);
        Pose3dStamped before = src.latest();
        r.status = "FAULT_BAD_READ";
        r.x = 9;                              // the driver would really repeat old values
        clk.now += 100000000L;
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
        assertSame("no new sample from a bad read", before, src.latest());
        assertEquals(1, src.getBadReads());
        clk.now += 500000000L;                // bad reads for 0.6 s in total
        assertEquals(BeaconStatus.LOST, loc.update().status);
        r.status = "READY";
        r.x = 0.01;
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
    }

    @Test
    public void faultsAndI2cExceptionsAreLostNeverThrown() throws Exception {
        FakeReader r = new FakeReader();
        PinpointPoseSource src = new PinpointPoseSource(r, new StepClock());
        BeaconLocalizer loc = new BeaconLocalizer(src, odo());
        loc.start(null);
        loc.setPose(0, 0, 0);
        loc.update();
        r.status = "FAULT_NO_PODS_DETECTED";
        assertEquals(BeaconStatus.LOST, loc.update().status);
        assertTrue(src.debugLine().contains("FAULT_NO_PODS_DETECTED"));
        r.status = "READY";
        r.throwNext = true;
        assertEquals(BeaconStatus.LOST, loc.update().status);
        assertEquals(1, src.getExceptions());
        assertTrue(src.getLastError().contains("I2C"));
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
    }

    @Test
    public void aPinpointRebootIsReconfiguredAndThePoseContinues() throws Exception {
        // Review finding 2 for this tier: a brown-out restarts the Pinpoint at 0,0,0 and
        // forgets its offsets. The old localizer would have jumped back to the start pose.
        FakeReader r = new FakeReader();
        StepClock sc = new StepClock();
        PinpointPoseSource src = new PinpointPoseSource(r, sc);
        BeaconLocalizer loc = new BeaconLocalizer(src, odo());
        loc.start(null);
        r.x = 1.0;
        loc.update();
        loc.setPose(2.0, 3.0, 0);
        sc.t += 1000000000L;
        r.x = 1.5;
        assertEquals(2.5, loc.update().x, 1e-9);
        r.status = "NOT_READY";               // reboot
        assertEquals(BeaconStatus.INITIALIZING, loc.update().status);
        r.status = "CALIBRATING";
        loc.update();
        r.status = "READY";                   // back, at its own origin
        r.x = 0;
        r.y = 0;
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals("continues from the last pose", 2.5, p.x, 1e-9);
        assertEquals(3.0, p.y, 1e-9);
        assertEquals(1, r.reconfigs);
        assertEquals(1, src.getReboots());
        assertEquals(1, loc.getWorldResets());
        assertTrue(loc.isRecovering());
        sc.t += 100000000L;                   // 0.1 m over 0.1 s
        r.x = 0.1;
        assertEquals(2.6, loc.update().x, 1e-9);
    }

    @Test
    public void anImpossibleJumpBetweenReadsIsAWorldReset() throws Exception {
        FakeReader r = new FakeReader();
        PinpointPoseSource src = new PinpointPoseSource(r, new StepClock());
        BeaconLocalizer loc = new BeaconLocalizer(src, odo());
        loc.start(null);
        loc.update();
        loc.setPose(0, 0, 0);
        r.x = 0.004;                          // 4 mm in 1 ms: fine (4 m/s)
        assertEquals(0.004, loc.update().x, 1e-9);
        assertEquals(0, src.getJumps());
        r.x = 2.0;                            // 2 m in 2 ms: impossible
        Pose2dStamped p = loc.update();
        assertEquals(1, src.getJumps());
        // continued from 0.004, extrapolated one read at the last velocity (2 m/s * 2 ms)
        assertEquals("the jump is not passed on as motion", 0.008, p.x, 1e-6);
        r.h = 3.0;                            // heading jump: also a reset
        loc.update();
        assertEquals(2, src.getJumps());
        assertEquals(2, loc.getWorldResets());
    }

    @Test
    public void sampleTimesAreStrictlyIncreasingAndMidRead() throws Exception {
        FakeReader r = new FakeReader();
        final long[] t = {5000};
        PinpointPoseSource src = started(new PinpointPoseSource(r,
                new PinpointPoseSource.TimeSource() {
                    public long nowNs() {
                        return t[0];   // a frozen clock
                    }
                }));
        src.poll();
        long a = src.latest().tNs;
        src.poll();
        assertTrue(src.latest().tNs > a);
        StepClock sc = new StepClock();
        PinpointPoseSource s2 = started(new PinpointPoseSource(r, sc));
        long before = sc.t;
        s2.poll();                            // clock read at +1 ms and +2 ms
        assertEquals(before + 1500000L, s2.latest().tNs);
    }
}
