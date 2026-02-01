package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class AimingUtil {
    public static final double TICKS_PER_REV = 28.0;
    public static final PIDFCoefficients LAUNCH_MOTOR_PID_COEFFICIENTS = new PIDFCoefficients(15, 0, 0, 21.5);
    static final double SPEED_MULT = 30.0;
    public static final double SERVO_MIN_DEG = 30.0;
    public static final double SERVO_MAX_DEG = 60.0;
    static final double MIN_SPEED = 0.5;

    static Pose2d storedPose = null;

    static Vector2d TARGET_POS = new Vector2d(64, -64);

    static double DistanceToRPM(double distance) {
        /* y=0.018024x^2+5.54937x+1956.51563 */
        return (0.018024 * distance * distance) + (5.54937 * distance) + 1956.51563;
    }

    static double DistanceToAngle(double distance, double min, double max) {
        /* y=0.000482561x^2+0.127878x+29.34231 */
        return Math.max(Math.min(0.000482561 * distance * distance + 0.127878 * distance + 29.34231, max), min);
    }
    static double getTargetVelocity(double target) {
        return ((target * TICKS_PER_REV) / 60.0);
    }

    static double getVelocityToAim(double targetX, double targetY, Pose2d pose) {
        double dx = targetX - pose.position.x;
        double dy = targetY - pose.position.y;

        double newHeading = Math.atan2(dy, dx);
        double error = Math.IEEEremainder(newHeading - pose.heading.toDouble(), 2.0 * Math.PI);

        return error / (2.0 * Math.PI) * SPEED_MULT;
    }

    static void setTeamSettings(Team team) {
        TARGET_POS = new Vector2d(TARGET_POS.x, Math.abs(TARGET_POS.y) * (team.ordinal() * 2 - 1));
    }

}
