package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class StreamCandidatesTest {

    @Test
    public void defaultOrderIsSdkAddressThenUsbDefaultThenMdns() {
        List<String> c = StreamCandidates.build("", "172.29.0.1");
        // The SDK-configured address equals the USB default here, so it appears once.
        assertEquals(Arrays.asList("http://172.29.0.1:5800/", "http://limelight.local:5800/"),
                c);
    }

    @Test
    public void configuredUrlComesFirstAndDuplicatesAreDropped() {
        List<String> c = StreamCandidates.build("http://10.0.0.5:5800/stream.mjpg",
                "172.28.0.1");
        assertEquals(Arrays.asList("http://10.0.0.5:5800/stream.mjpg",
                "http://172.28.0.1:5800/", "http://172.29.0.1:5800/",
                "http://limelight.local:5800/"), c);
        assertEquals(StreamCandidates.build("http://172.29.0.1:5800/", null),
                Arrays.asList("http://172.29.0.1:5800/", "http://limelight.local:5800/"));
    }

    @Test
    public void bareHostOrHostPortInConfigIsCompleted() {
        assertEquals("http://172.29.0.1:5800/",
                StreamCandidates.build("172.29.0.1", null).get(0));
        assertEquals("http://172.29.0.1:5802/",
                StreamCandidates.build("172.29.0.1:5802", null).get(0));
        assertEquals("http://limelight.local:5800/x",
                StreamCandidates.build("limelight.local:5800/x", null).get(0));
    }

    @Test
    public void limelightAddressDerivedLikeTheSdkFromTheHubSideAddress() {
        // EthernetOverUsbSerialNumber.toString() is "<iface>:<hub ip>"; the SDK's default
        // Limelight address is the hub's address with the last octet set to 1.
        assertEquals("172.29.0.1", StreamCandidates.limelightIpFromConnectionInfo(
                "eth0:172.29.0.23"));
        assertEquals("172.29.0.1", StreamCandidates.limelightIpFromConnectionInfo(
                "172.29.0.23"));
        assertNull(StreamCandidates.limelightIpFromConnectionInfo("eth0:fe80::1"));
        assertNull(StreamCandidates.limelightIpFromConnectionInfo(null));
        assertNull(StreamCandidates.limelightIpFromConnectionInfo("garbage"));
    }

    @Test
    public void pathVariantsOnlyForRootUrls() {
        assertEquals(Arrays.asList("http://h:5800/stream.mjpg", "http://h:5800/?action=stream"),
                StreamCandidates.pathVariants("http://h:5800/"));
        assertEquals(0, StreamCandidates.pathVariants("http://h:5800/stream.mjpg").size());
    }
}
