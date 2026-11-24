package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import org.junit.Test;

public class BeaconConfigTest {

    private static void assertRejected(String json, String mustMention) {
        try {
            BeaconConfig.parse(json);
            fail("expected BeaconException for: " + json);
        } catch (BeaconException e) {
            assertTrue("message \"" + e.getMessage() + "\" should mention " + mustMention,
                    e.getMessage().contains(mustMention));
        }
    }

    @Test
    public void parsesAFullFile() throws Exception {
        String json = "{\n"
                + "  \"tier\": \"fake\",\n"
                + "  \"T_robot_cam\": {\"x\": 0.1, \"y\": -0.02, \"z\": 0.25, "
                + "\"roll\": 0.0, \"pitch\": 0.2618, \"yaw\": 0.1},\n"
                + "  \"fix\": {\"enabled\": false, \"gainXY\": 0.5, \"gainHeading\": 0.25, "
                + "\"maxJumpM\": 0.75, \"maxJumpRad\": 0.4, \"recoveryWindowS\": 0.8, "
                + "\"maxSpeedMps\": 0.6, \"maxOmegaRadPs\": 0.3},\n"
                + "  \"limelight\": {\"streamUrl\": \"172.29.0.1\", \"latencyMs\": 25},\n"
                + "  \"log\": {\"enabled\": true, \"dir\": \"/tmp/x\"},\n"
                + "  \"sourceTimeoutS\": 0.25\n}";
        BeaconConfig c = BeaconConfig.parse(json);
        assertEquals("fake", c.tier);
        assertEquals(0.1, c.cameraMount.x, 0);
        assertEquals(-0.02, c.cameraMount.y, 0);
        assertEquals(0.25, c.cameraMount.z, 0);
        assertEquals(0.2618, c.cameraMount.pitch, 0);
        assertFalse(c.fix.enabled);
        assertEquals(0.5, c.fix.gainXY, 0);
        assertEquals(0.25, c.fix.gainHeading, 0);
        assertEquals(0.75, c.fix.maxJumpM, 0);
        assertEquals(0.4, c.fix.maxJumpRad, 0);
        assertEquals(0.8, c.fix.recoveryWindowS, 0);
        assertEquals(0.6, c.fix.maxSpeedMps, 0);
        assertEquals(0.3, c.fix.maxOmegaRadPs, 0);
        assertEquals("172.29.0.1", c.stream.streamUrl);
        assertEquals(25, c.stream.latencyMs, 0);
        assertTrue(c.log.enabled);
        assertEquals("/tmp/x", c.log.dir);
        assertEquals(0.25, c.sourceTimeoutS, 0);
        assertTrue(c.warnings.isEmpty());
    }

    @Test
    public void emptyObjectGivesOdometryDefaultsWithoutAskingForCameraAngles() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{}");
        BeaconConfig d = BeaconConfig.defaults();
        assertEquals("odometry", c.tier);
        assertEquals(d.tier, c.tier);
        assertEquals(d.fix.gainXY, c.fix.gainXY, 0);
        assertEquals(0.0, c.cameraMount.x, 0);
        assertTrue("no T_robot_cam nagging for the odometry tier: " + c.warnings,
                c.warnings.isEmpty());
    }

    @Test
    public void cameraMountIsUsedByTheOdometryTierNow() throws Exception {
        // Beacon's vision core needs T_robot_cam (the Limelight mount), so it is no longer
        // flagged as ignored for the odometry tier.
        BeaconConfig c = BeaconConfig.parse("{\"T_robot_cam\":{\"x\":0.1}}");
        assertTrue(c.warnings.toString(), c.warnings.isEmpty());
        assertEquals(0.1, c.cameraMount.x, 0);
    }

    @Test
    public void configTagSectionRejectedWithHint() {
        assertRejected("{\"tag\":{\"gainXY\":0.3}}", "\"fix\"");
        assertRejected("{\"tag\":{}}", "renamed");
    }

    @Test
    public void parkedMegaTagKeysOnlyWarn() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\"limelight\":{\"pipeline\":2,\"minTags\":1}}");
        assertEquals(2, c.warnings.size());
        assertTrue(c.warnings.get(0), c.warnings.get(0).contains("AprilTag"));
    }

    @Test
    public void fixHistoryAndPriorsParseAndValidate() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\"fix\":{\"historyS\":1.5,"
                + "\"priorSigmaXYM\":0.03,\"priorSigmaHeadingRad\":0.01}}");
        assertEquals(1.5, c.fix.historyS, 0);
        assertEquals(0.03, c.fix.priorSigmaXYM, 0);
        assertEquals(0.01, c.fix.priorSigmaHeadingRad, 0);
        assertEquals(0.75, BeaconConfig.defaults().fix.historyS, 0);
        assertRejected("{\"fix\":{\"historyS\":0.01}}", "fix.historyS");
        assertRejected("{\"fix\":{\"priorSigmaXYM\":0}}", "fix.priorSigmaXYM");
    }

    @Test
    public void missingFileIsDefaultsPlusWarningNotAnException() throws Exception {
        File f = new File(System.getProperty("java.io.tmpdir"), "no_such_beacon_" + System.nanoTime()
                + ".json");
        BeaconConfig c = BeaconConfig.load(f);
        assertEquals("odometry", c.tier);
        assertEquals(1, c.warnings.size());
        assertTrue(c.warnings.get(0).contains("beacon.json"));
        // null path behaves the same
        assertEquals(1, BeaconConfig.load(null).warnings.size());
    }

    @Test
    public void loadsFromDisk() throws Exception {
        File f = File.createTempFile("beacon", ".json");
        f.deleteOnExit();
        FileWriter w = new FileWriter(f);
        try {
            w.write("{\"tier\":\"fake\",\"T_robot_cam\":{\"x\":0.3}}");
        } finally {
            w.close();
        }
        BeaconConfig c = BeaconConfig.load(f);
        assertEquals("fake", c.tier);
        assertEquals(0.3, c.cameraMount.x, 0);
    }

    @Test
    public void unknownKeysInsideASectionWarn() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":{\"x\":0,"
                + "\"yawn\":1},\"fix\":{\"gainxy\":1},\"limelight\":{\"pipline\":1}}");
        String all = c.warnings.toString();
        assertTrue(all, all.contains("T_robot_cam.yawn"));
        assertTrue(all, all.contains("fix.gainxy"));
        assertTrue(all, all.contains("limelight.pipline"));
    }

    @Test
    public void unknownTopLevelSectionIsAnError() {
        // finding 11: a misspelled section used to be silently ignored
        assertRejected("{\"tagg\":{}}", "tagg");
        assertRejected("{\"T_robot_camera\":{\"x\":0.1}}", "T_robot_camera");
        assertRejected("{\"vins\":{\"configPath\":\"a\"}}", "vins");
    }

    @Test
    public void byteOrderMarkAndCommentsAreAccepted() throws Exception {
        BeaconConfig c = BeaconConfig.parse("\uFEFF{ // Beacon settings\n"
                + "  \"fix\": { \"gainXY\": 0.4 }  // a comment after a value\n}\n");
        assertEquals(0.4, c.fix.gainXY, 0);
        // a "//" inside a string is text, not a comment
        assertEquals("a//b", BeaconConfig.parse("{\"log\":{\"dir\":\"a//b\"}}").log.dir);
    }

    @Test
    public void gainsOutsideZeroOneAreRejected() {
        assertRejected("{\"fix\":{\"gainXY\":1.5}}", "fix.gainXY");
        assertRejected("{\"fix\":{\"gainXY\":-0.1}}", "fix.gainXY");
        assertRejected("{\"fix\":{\"gainHeading\":2}}", "fix.gainHeading");
    }

    @Test
    public void gainsAtTheEdgesAreAccepted() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\"fix\":{\"gainXY\":0,\"gainHeading\":1}}");
        assertEquals(0, c.fix.gainXY, 0);
        assertEquals(1, c.fix.gainHeading, 0);
    }

    @Test
    public void implausibleLeverArmIsRejected() {
        assertRejected("{\"T_robot_cam\":{\"x\":150}}", "T_robot_cam position");   // millimetres
        assertRejected("{\"T_robot_cam\":{\"x\":0.8,\"y\":0.8}}", "T_robot_cam position");
    }

    @Test
    public void anglesInDegreesAreCaught() {
        assertRejected("{\"T_robot_cam\":{\"pitch\":15}}", "radians");
        assertRejected("{\"T_robot_cam\":{\"yaw\":-90}}", "radians");
    }

    @Test
    public void smallDegreeValuesAreCaughtToo() throws Exception {
        // finding 5: "pitch": 5 (degrees) used to pass. Pitch is limited to +/- pi/2 ...
        assertRejected("{\"T_robot_cam\":{\"pitch\":5}}", "radians");
        assertRejected("{\"T_robot_cam\":{\"pitch\":2}}", "radians");
        // ... and a whole number of radians >= 2 on roll/yaw is flagged
        BeaconConfig c = BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":{\"yaw\":3}}");
        assertTrue(c.warnings.toString(), c.warnings.toString().contains("degrees"));
        assertTrue(BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":{\"yaw\":1.5708,"
                + "\"pitch\":-1.5}}").warnings.isEmpty());
    }

    @Test
    public void nonFiniteNumbersAreRejected() {
        assertRejected("{\"T_robot_cam\":{\"x\":1e999}}", "finite");
        assertRejected("{\"T_robot_cam\":{\"yaw\":NaN}}", "JSON error");   // not valid JSON
    }

    @Test
    public void unknownTierIsRejected() {
        assertRejected("{\"tier\":\"orbital\"}", "tier");
        assertRejected("{\"tier\":\"vins\"}", "parked");
        assertRejected("{\"tier\":\"arcore\"}", "parked");
    }

    @Test
    public void wrongTypesAreRejectedWithTheKeyName() {
        assertRejected("{\"fix\":{\"enabled\":\"yes\"}}", "fix.enabled");
        assertRejected("{\"T_robot_cam\":{\"x\":\"0.1\"}}", "T_robot_cam.x");
        assertRejected("{\"T_robot_cam\":3}", "T_robot_cam");
        assertRejected("{\"tier\":5}", "tier");
        assertRejected("[1,2]", "JSON object");
    }

    @Test
    public void otherRangesAreChecked() {
        assertRejected("{\"fix\":{\"maxJumpM\":0}}", "fix.maxJumpM");
        assertRejected("{\"fix\":{\"maxJumpRad\":4}}", "fix.maxJumpRad");
        assertRejected("{\"fix\":{\"recoveryWindowS\":-1}}", "fix.recoveryWindowS");
        assertRejected("{\"fix\":{\"maxSpeedMps\":0}}", "fix.maxSpeedMps");
        assertRejected("{\"fix\":{\"maxOmegaRadPs\":0}}", "fix.maxOmegaRadPs");
        assertRejected("{\"limelight\":{\"readTimeoutMs\":50}}", "limelight.readTimeoutMs");
        assertRejected("{\"limelight\":{\"connectTimeoutMs\":400.5}}", "whole number");
        assertRejected("{\"limelight\":{\"latencyMs\":-2}}", "limelight.latencyMs");
        assertRejected("{\"limelight\":{\"defaultLatencyMs\":600}}",
                "limelight.defaultLatencyMs");
        assertRejected("{\"limelight\":{\"connectTimeoutMs\":20000}}",
                "limelight.connectTimeoutMs");
        assertRejected("{\"sourceTimeoutS\":0}", "sourceTimeoutS");
        assertRejected("{\"log\":{\"dir\":\"\"}}", "log.dir");
    }

    @Test
    public void malformedJsonGivesPositionInMessage() {
        assertRejected("{\"tier\": \"fake\",}", "line 1");
        assertRejected("{\"tier\": \"fake\"", "end of file");
        assertRejected("{\"a\":1,\"a\":2}", "duplicate");
        assertRejected("{\"tier\": 'fake'}", "unexpected character");
        assertRejected("{} extra", "after the JSON value");
        assertRejected("{\n\n  \"x\": 01 }", "line 3");
    }

    @Test
    public void parserHandlesEscapesNumbersAndNesting() throws Exception {
        Object v = MiniJson.parse("{\"a\":[1,2.5e-1,-3,true,false,null,\"x\\n\\u0041\"],\"b\":{}}");
        assertNotNull(v);
        BeaconConfig c = BeaconConfig.parse("{\"log\":{\"dir\":\"C:\\\\a\\/b\"}}");
        assertEquals("C:\\a/b", c.log.dir);
    }

    @Test
    public void mountTransformMatchesFrameMath() throws Exception {
        BeaconConfig c = BeaconConfig.parse(
                "{\"T_robot_cam\":{\"x\":0.1,\"y\":0.2,\"z\":0.3,\"roll\":0.1,\"pitch\":0.2,\"yaw\":0.3}}");
        double[] a = c.cameraMount.toTransform();
        double[] b = new double[7];
        FrameMath.robotFromCamera(0.1, 0.2, 0.3, 0.1, 0.2, 0.3, b);
        for (int i = 0; i < 7; i++) {
            assertEquals(b[i], a[i], 0);
        }
    }

    @Test
    public void directoryPathIsTreatedAsMissing() throws IOException {
        File dir = File.createTempFile("beacon", "dir");
        assertTrue(dir.delete());
        assertTrue(dir.mkdir());
        dir.deleteOnExit();
        // a directory is not a file: treated as missing -> defaults + warning
        try {
            assertEquals(1, BeaconConfig.load(dir).warnings.size());
        } catch (BeaconException e) {
            fail(e.getMessage());
        }
    }
}
