package org.firstinspires.ftc.teamcode.beacon;

/** Checked exception for configuration and pose-source start-up failures. */
public class BeaconException extends Exception {
    public BeaconException(String message) {
        super(message);
    }

    public BeaconException(String message, Throwable cause) {
        super(message, cause);
    }
}
