package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class AngleTesting extends LinearOpMode {

    final double GEAR_RATIO = 29.0 / 300.0;

    final double SERVO_MIN_DEG = 30.0;
    final double SERVO_MAX_DEG = 60.0;

    Servo launcherAngleServo;

    @Override
    public void runOpMode() throws InterruptedException {

        launcherAngleServo = hardwareMap.get(Servo.class, "launcherAngleServo");

        final double LAUNCHER_ANGLE_MIN = SERVO_MIN_DEG * GEAR_RATIO;
        final double LAUNCHER_ANGLE_MAX = SERVO_MAX_DEG * GEAR_RATIO;

        telemetry.addData("Launcher Min Angle", LAUNCHER_ANGLE_MIN);
        telemetry.addData("Launcher Max Angle", LAUNCHER_ANGLE_MAX);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {

            double stick = (-gamepad1.right_stick_y + 1.0) / 2.0;

            double launcherAngle = LAUNCHER_ANGLE_MIN + stick * (LAUNCHER_ANGLE_MAX - LAUNCHER_ANGLE_MIN);

            double servoAngle = launcherAngle / GEAR_RATIO;

            double servoPos = Math.max(0.0, Math.min(1.0, servoAngle / 180.0));

            launcherAngleServo.setPosition(servoPos);

            telemetry.addData("Stick", stick);
            telemetry.addData("Launcher Angle (deg)", launcherAngle);
            telemetry.addData("Servo Angle (deg)", servoAngle);
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}
