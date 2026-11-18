package org.firstinspires.ftc.teamcode.beacon.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Where the Limelight 3A's camera stream might be, in the order they are tried. Nothing here
 * has been confirmed on a Control Hub yet; the probe OpMode reports which one worked.
 *
 * <h3>Sources for each candidate</h3>
 * <ol>
 *   <li>{@code limelight.streamUrl} from beacon.json, if set (the team pins the one that
 *       worked).</li>
 *   <li>The Limelight address in the Robot Controller configuration, port 5800. The SDK
 *       (Hardware 12.0.0, {@code Limelight3A}) talks to the Limelight's REST API at
 *       {@code http://<that address>:5807}. The address is stored per device in the RC
 *       configuration; when the device is first added, the SDK
 *       ({@code EthernetOverUsbConfiguration}) defaults it to the Hub's own address on the
 *       Limelight's USB-Ethernet link with the last number set to 1 (Hub 172.29.0.23 gives
 *       Limelight 172.29.0.1).</li>
 *   <li>{@code http://172.29.0.1:5800/}: the robot-side Limelight address in Limelight's
 *       documentation; 5800 is the camera stream port in Limelight's port-forwarding
 *       example.</li>
 *   <li>{@code http://limelight.local:5800/}: the name a laptop uses. Android usually cannot
 *       resolve {@code .local} names, and the lookup may take seconds, so it is tried last.</li>
 * </ol>
 * If a root URL answers with something that is not an MJPEG stream (for example an HTML
 * page), {@link #pathVariants} gives the usual stream paths on the same server.
 */
public final class StreamCandidates {
    public static final int STREAM_PORT = 5800;
    public static final String USB_DEFAULT_IP = "172.29.0.1";
    public static final String MDNS_HOST = "limelight.local";

    private StreamCandidates() {}

    /**
     * @param configured  beacon.json {@code limelight.streamUrl} ("" or null = none); a bare
     *                    host or host:port is completed to {@code http://host:5800/}
     * @param sdkLimelightIp the Limelight address from the RC configuration, or null
     */
    public static List<String> build(String configured, String sdkLimelightIp) {
        List<String> out = new ArrayList<String>();
        if (configured != null && configured.trim().length() > 0) {
            add(out, complete(configured.trim()));
        }
        if (sdkLimelightIp != null && sdkLimelightIp.length() > 0) {
            add(out, "http://" + sdkLimelightIp + ":" + STREAM_PORT + "/");
        }
        add(out, "http://" + USB_DEFAULT_IP + ":" + STREAM_PORT + "/");
        add(out, "http://" + MDNS_HOST + ":" + STREAM_PORT + "/");
        return out;
    }

    private static void add(List<String> out, String url) {
        if (!out.contains(url)) {
            out.add(url);
        }
    }

    /** "172.29.0.1" -> "http://172.29.0.1:5800/", "h:5802" -> "http://h:5802/". */
    static String complete(String s) {
        String u = s.startsWith("http://") ? s : "http://" + s;
        int hostStart = "http://".length();
        int slash = u.indexOf('/', hostStart);
        String host = slash < 0 ? u.substring(hostStart) : u.substring(hostStart, slash);
        String path = slash < 0 ? "/" : u.substring(slash);
        if (host.indexOf(':') < 0) {
            host = host + ":" + STREAM_PORT;
        }
        return "http://" + host + path;
    }

    /** Other common MJPEG paths on the same server, for a URL whose path is "/". */
    public static List<String> pathVariants(String url) {
        List<String> out = new ArrayList<String>();
        int hostStart = url.indexOf("://");
        int slash = hostStart < 0 ? -1 : url.indexOf('/', hostStart + 3);
        if (slash >= 0 && slash == url.length() - 1) {
            out.add(url + "stream.mjpg");
            out.add(url + "?action=stream");
        }
        return out;
    }

    /**
     * The SDK's default Limelight address from a Limelight3A's connection info. The SDK shows
     * {@code EthernetOverUsbSerialNumber.toString()} there, "{@code <iface>:<hub ip>}"; the
     * default Limelight address is the Hub address with its last number set to 1. Null if
     * the text is not an IPv4 address.
     */
    public static String limelightIpFromConnectionInfo(String info) {
        if (info == null) {
            return null;
        }
        String ip = info.substring(info.lastIndexOf(':') + 1).trim();
        String[] parts = ip.split("\\.");
        if (parts.length != 4) {
            return null;
        }
        for (String p : parts) {
            if (p.length() == 0 || p.length() > 3) {
                return null;
            }
            for (int i = 0; i < p.length(); i++) {
                if (!Character.isDigit(p.charAt(i))) {
                    return null;
                }
            }
            if (Integer.parseInt(p) > 255) {
                return null;
            }
        }
        return parts[0] + "." + parts[1] + "." + parts[2] + ".1";
    }
}
