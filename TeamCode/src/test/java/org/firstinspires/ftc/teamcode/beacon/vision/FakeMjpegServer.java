package org.firstinspires.ftc.teamcode.beacon.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

/**
 * A tiny local HTTP server that behaves like a camera's MJPEG endpoint, for tests. Paths:
 * {@code mjpegPath} serves multipart/x-mixed-replace fake-JPEG parts (see TestImages) at {@code fps};
 * {@code "/"} (if different) serves an HTML page, like a device's web UI; anything else is 404.
 * {@link #stop()} closes every socket (like unplugging the device); {@link #start()} again
 * rebinds the same port.
 */
final class FakeMjpegServer {
    private static final Charset ASCII = Charset.forName("US-ASCII");

    final String mjpegPath;
    final int width, height;
    volatile double fps;
    volatile boolean sendContentLength = true;
    volatile boolean paused;
    final AtomicLong framesSent = new AtomicLong();
    final AtomicLong connections = new AtomicLong();

    private int port;
    private volatile ServerSocket server;
    private final List<Socket> clients = new ArrayList<Socket>();
    private volatile boolean running;
    private final byte[][] jpegs = new byte[8][];

    FakeMjpegServer(String mjpegPath, int width, int height, double fps) {
        this.mjpegPath = mjpegPath;
        this.width = width;
        this.height = height;
        this.fps = fps;
        for (int i = 0; i < jpegs.length; i++) {
            jpegs[i] = TestImages.fakeJpegPattern(width, height, 16 * i + 8);
        }
    }

    int port() {
        return port;
    }

    String url(String path) {
        return "http://127.0.0.1:" + port + path;
    }

    synchronized void start() throws IOException {
        ServerSocket s = new ServerSocket();
        s.setReuseAddress(true);
        s.bind(new InetSocketAddress(InetAddress.getByName("127.0.0.1"), port));
        port = s.getLocalPort();
        server = s;
        running = true;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                acceptLoop();
            }
        }, "FakeMjpegAccept");
        t.setDaemon(true);
        t.start();
    }

    synchronized void stop() {
        running = false;
        try {
            if (server != null) {
                server.close();
            }
        } catch (IOException ignored) {
            // closing anyway
        }
        for (Socket c : clients) {
            try {
                c.close();
            } catch (IOException ignored) {
                // closing anyway
            }
        }
        clients.clear();
    }

    private void acceptLoop() {
        ServerSocket s = server;
        while (running) {
            final Socket c;
            try {
                c = s.accept();
            } catch (IOException e) {
                return;
            }
            synchronized (this) {
                if (!running) {
                    try {
                        c.close();
                    } catch (IOException ignored) {
                        // closing anyway
                    }
                    return;
                }
                clients.add(c);
            }
            connections.incrementAndGet();
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    serve(c);
                }
            }, "FakeMjpegClient");
            t.setDaemon(true);
            t.start();
        }
    }

    private void serve(Socket c) {
        try {
            InputStream in = c.getInputStream();
            String requestLine = readLine(in);
            String line;
            do {
                line = readLine(in);
            } while (line != null && line.length() > 0);
            String path = requestLine == null ? "/" : requestLine.split(" ")[1];
            OutputStream out = c.getOutputStream();
            if (path.equals(mjpegPath)) {
                out.write(("HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; "
                        + "boundary=frameboundary\r\nCache-Control: no-cache\r\n\r\n")
                        .getBytes(ASCII));
                out.flush();
                long n = 0;
                while (running && !c.isClosed()) {
                    if (paused) {
                        Thread.sleep(5);
                        continue;
                    }
                    byte[] j = jpegs[(int) (n % jpegs.length)];
                    StringBuilder h = new StringBuilder();
                    h.append("--frameboundary\r\nContent-Type: image/jpeg\r\n");
                    if (sendContentLength) {
                        h.append("Content-Length: ").append(j.length).append("\r\n");
                    }
                    h.append("X-Timestamp: ").append(n).append(".000000\r\n\r\n");
                    out.write(h.toString().getBytes(ASCII));
                    out.write(j);
                    out.write("\r\n".getBytes(ASCII));
                    out.flush();
                    framesSent.incrementAndGet();
                    n++;
                    Thread.sleep((long) (1000 / fps));
                }
            } else if (path.equals("/")) {
                out.write("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n<html>ui</html>"
                        .getBytes(ASCII));
                out.flush();
            } else {
                out.write("HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\n\r\nno"
                        .getBytes(ASCII));
                out.flush();
            }
        } catch (IOException e) {
            // client went away or we were stopped
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            try {
                c.close();
            } catch (IOException ignored) {
                // closing anyway
            }
        }
    }

    private static String readLine(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder();
        int b;
        while ((b = in.read()) >= 0) {
            if (b == '\n') {
                break;
            }
            if (b != '\r') {
                sb.append((char) b);
            }
        }
        return (b < 0 && sb.length() == 0) ? null : sb.toString();
    }
}
