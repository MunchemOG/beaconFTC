package org.firstinspires.ftc.teamcode.beacon.opmodes;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconException;

import java.util.List;

/** Small helpers shared by the Beacon OpModes. */
final class Support {
    private Support() {}

    /**
     * Loads /sdcard/FIRST/beacon/beacon.json. A broken file does not leave the robot without
     * a localizer: the built-in defaults are used and the problem goes into {@code notes} for
     * telemetry. A tier other than "odometry" is overridden for the same reason.
     */
    static BeaconConfig loadConfig(List<String> notes) {
        BeaconConfig cfg;
        try {
            cfg = BeaconConfig.loadDefault();
        } catch (BeaconException e) {
            notes.add("beacon.json REJECTED, using defaults: " + e.getMessage());
            return BeaconConfig.defaults();
        }
        if (!BeaconConfig.TIER_ODOMETRY.equals(cfg.tier)) {
            notes.add("beacon.json tier is \"" + cfg.tier + "\"; this OpMode runs the odometry "
                    + "tier, so defaults are used. Set \"tier\": \"odometry\".");
            return BeaconConfig.defaults();
        }
        return cfg;
    }
}
