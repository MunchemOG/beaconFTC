package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

/** The stream / record / probe sections added for the Limelight frame path (task J3). */
public class BeaconConfigVisionTest {

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
    public void defaultsTryEveryCandidateAndEstimateLatency() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{}");
        assertEquals("", c.stream.streamUrl);
        assertTrue(c.stream.connectTimeoutMs >= 100);
        assertTrue(c.stream.readTimeoutMs >= c.stream.connectTimeoutMs);
        assertEquals(-1, c.stream.latencyMs, 0);
        assertTrue(c.stream.defaultLatencyMs > 0);
        assertEquals("/sdcard/FIRST/beacon/runs", c.record.dir);
        assertTrue(c.record.maxHz > 0);
        assertTrue(c.record.frameQueue >= 2);
        assertEquals("/sdcard/FIRST/beacon/field.json", c.record.fieldFile);
        assertNull(c.probe.start);
        assertTrue(c.probe.marks.isEmpty());
    }

    @Test
    public void parsesStreamRecordAndProbeSections() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\n"
                + "\"limelight\": {\"streamUrl\": \"http://172.29.0.1:5800/\", "
                + "\"connectTimeoutMs\": 250, \"readTimeoutMs\": 800, \"latencyMs\": 27.5, "
                + "\"defaultLatencyMs\": 40},\n"
                + "\"record\": {\"dir\": \"/sdcard/x\", \"maxHz\": 15, \"frameQueue\": 4, "
                + "\"fieldFile\": \"/sdcard/f.json\", \"checkerboardHz\": 2},\n"
                + "\"probe\": {\"start\": [-1.5, 0.9, 0.0], "
                + "\"marks\": [[0.6096, 0.6096, 0], [-1.2, 1.2, 1.5708]]}\n}");
        assertEquals("http://172.29.0.1:5800/", c.stream.streamUrl);
        assertEquals(250, c.stream.connectTimeoutMs);
        assertEquals(800, c.stream.readTimeoutMs);
        assertEquals(27.5, c.stream.latencyMs, 0);
        assertEquals(40, c.stream.defaultLatencyMs, 0);
        assertEquals("/sdcard/x", c.record.dir);
        assertEquals(15, c.record.maxHz, 0);
        assertEquals(4, c.record.frameQueue);
        assertEquals("/sdcard/f.json", c.record.fieldFile);
        assertEquals(2, c.record.checkerboardHz, 0);
        assertEquals(-1.5, c.probe.start[0], 0);
        assertEquals(0.9, c.probe.start[1], 0);
        assertEquals(2, c.probe.marks.size());
        assertEquals(1.5708, c.probe.marks.get(1)[2], 0);
        assertTrue(c.warnings.toString(), c.warnings.isEmpty());
    }

    @Test
    public void badValuesAreRejectedWithTheirName() {
        assertRejected("{\"limelight\": {\"readTimeoutMs\": 5}}", "limelight.readTimeoutMs");
        assertRejected("{\"limelight\": {\"latencyMs\": 900}}", "limelight.latencyMs");
        assertRejected("{\"limelight\": {\"streamUrl\": 5}}", "limelight.streamUrl");
        assertRejected("{\"limelight\": {\"streamUrl\": \"ftp://x\"}}", "limelight.streamUrl");
        assertRejected("{\"record\": {\"maxHz\": 0}}", "record.maxHz");
        assertRejected("{\"record\": {\"frameQueue\": 1}}", "record.frameQueue");
        assertRejected("{\"probe\": {\"start\": [1, 2]}}", "probe.start");
        assertRejected("{\"probe\": {\"marks\": [[1, 2, \"a\"]]}}", "probe.marks[0]");
        assertRejected("{\"probe\": {\"marks\": [[1, 2, 3]], \"start\": [9, 0, 0]}}",
                "probe.start");
        assertRejected("{\"probe\": {\"start\": [0, 0, 7]}}", "probe.start");
    }
}
