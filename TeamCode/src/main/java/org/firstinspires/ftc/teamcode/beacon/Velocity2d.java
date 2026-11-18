package org.firstinspires.ftc.teamcode.beacon;

/**
 * Immutable planar velocity in the ROBOT frame: {@code vx} forward and {@code vy} left in
 * metres per second, {@code omega} counter-clockwise in radians per second.
 */
public final class Velocity2d {
    public static final Velocity2d ZERO = new Velocity2d(0, 0, 0, 0);

    public final double vx;
    public final double vy;
    public final double omega;
    public final long tNs;

    public Velocity2d(double vx, double vy, double omega, long tNs) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        this.tNs = tNs;
    }
}
