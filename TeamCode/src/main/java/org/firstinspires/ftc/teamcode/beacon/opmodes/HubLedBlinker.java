package org.firstinspires.ftc.teamcode.beacon.opmodes;

import android.os.SystemClock;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.beacon.vision.LedLatencyMeter;

import java.util.Collection;
import java.util.List;
import java.util.Random;

/**
 * Blinks the Control Hub's status LED (green / off) at random intervals of 350-650 ms on its
 * own thread, telling a {@link LedLatencyMeter} the Hub time of every change. The time used is
 * when {@code setConstant} RETURNED (the SDK sends a ping and the LED command to the Hub's
 * Lynx board and waits for each); the LED itself may change slightly before that, which makes
 * the measured latency read short by at most {@link #getLastCommandMs()}.
 *
 * <p>The LED's normal pattern is saved first and put back by {@link #stop()}.
 */
final class HubLedBlinker {
    private static final int GREEN = 0xFF00FF00;
    private static final int BLACK = 0xFF000000;

    private final LynxModule hub;
    private final LedLatencyMeter meter;
    private volatile boolean running;
    private volatile double lastCommandMs = Double.NaN;
    private volatile String error = "";
    private Thread thread;
    private Collection<Blinker.Step> saved;

    private HubLedBlinker(LynxModule hub, LedLatencyMeter meter) {
        this.hub = hub;
        this.meter = meter;
    }

    /** The Control Hub's own module (embedded), or the first one; null if none. */
    static HubLedBlinker create(HardwareMap hw, LedLatencyMeter meter) {
        LynxModule pick = null;
        try {
            List<LynxModule> all = hw.getAll(LynxModule.class);
            for (LynxModule m : all) {
                if (LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) {
                    pick = m;
                    break;
                }
            }
            if (pick == null && !all.isEmpty()) {
                pick = all.get(0);
            }
        } catch (RuntimeException e) {
            return null;
        }
        return pick == null ? null : new HubLedBlinker(pick, meter);
    }

    synchronized void start() {
        if (running) {
            return;
        }
        meter.reset();
        try {
            saved = hub.getPattern();
        } catch (RuntimeException e) {
            saved = null;
        }
        running = true;
        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                loop();
            }
        }, "BeaconLedBlink");
        thread.setDaemon(true);
        thread.start();
    }

    synchronized void stop() {
        if (!running) {
            return;
        }
        running = false;
        Thread t = thread;
        thread = null;
        if (t != null) {
            t.interrupt();
            try {
                t.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        try {
            if (saved != null) {
                hub.setPattern(saved);
            }
        } catch (RuntimeException e) {
            error = "could not restore the LED pattern: " + e;
        }
    }

    boolean isRunning() {
        return running;
    }

    /** Duration of the last setConstant call (ms): bound on the LED timing error. */
    double getLastCommandMs() {
        return lastCommandMs;
    }

    String getError() {
        return error;
    }

    private void loop() {
        Random rnd = new Random();
        boolean on = false;
        while (running) {
            try {
                Thread.sleep(350 + rnd.nextInt(300));
            } catch (InterruptedException e) {
                return;
            }
            if (!running) {
                return;
            }
            boolean next = !on;
            long t0 = SystemClock.elapsedRealtimeNanos();
            try {
                hub.setConstant(next ? GREEN : BLACK);
            } catch (RuntimeException e) {
                error = "LED command failed: " + e;
                continue;
            }
            long t1 = SystemClock.elapsedRealtimeNanos();
            on = next;
            lastCommandMs = (t1 - t0) / 1e6;
            meter.onLedChange(t1, on);
        }
    }
}
