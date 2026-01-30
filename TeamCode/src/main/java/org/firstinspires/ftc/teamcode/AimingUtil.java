package org.firstinspires.ftc.teamcode;

public class AimingUtil {
    public static final double TICKS_PER_REV = 28.0;
    static double DistanceToRPM(double distance) {
        /* y=0.018024x^2+5.54937x+1956.51563 */
        return 0.018024 * distance * distance + 5.54937 * distance + 1956.51563;
    }

    static double DistanceToAngle(double distance, double min, double max) {
        /* y=0.000482561x^2+0.127878x+29.34231 */
        return Math.max(Math.min(0.000482561 * distance * distance + 0.127878 * distance + 29.34231, max), min);
    }
    static double getTargetVelocity(double target) {
        return (target * TICKS_PER_REV) / 60.0;
    }

}
