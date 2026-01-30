package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class AimingUtil {
    public static final double TICKS_PER_REV = 28.0;
    static final double SPEED_MULT = 30.0;

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

    static double getVelocityToAim(double targetX, double targetY, Pose2d pose) {

//        double dx = targetX - pose.position.x;
//        double dy = targetY - pose.position.y;
//
//        double newHeading = Math.atan2(dy, dx);
//        double error = Math.IEEEremainder(newHeading - pose.heading.toDouble(), 2.0 * Math.PI);
//
//        return error / (2.0 * Math.PI) * SPEED_MULT;


        double dx = targetX - pose.position.x;
        double dy = targetY - pose.position.y;

        double newHeading = Math.atan2(dy, dx);
        double error = Math.IEEEremainder(newHeading - pose.heading.toDouble(), 2.0 * Math.PI);

        double clampedError = Math.max(
                -Math.PI / 2,
                Math.min(Math.PI / 2, error)
        );

        double sinVal = Math.sin(clampedError);

        double epsilon = 0.05;
        sinVal = Math.copySign(Math.max(Math.abs(sinVal), epsilon), sinVal);

        return SPEED_MULT * clampedError / sinVal;
    }

}
