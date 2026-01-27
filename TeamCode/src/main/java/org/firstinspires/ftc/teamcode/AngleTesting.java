package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class AngleTesting extends LinearOpMode {

    final double ANGLE_MIN = 30.0 / 180.0;
    final double ANGLE_MAX = 60.0 / 180.0;

    Servo launcherAngleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        launcherAngleServo = hardwareMap.get(Servo.class, "launcherAngleServo");
        launcherAngleServo.scaleRange(ANGLE_MIN, ANGLE_MAX);

        waitForStart();

        while (!isStopRequested()) {
            launcherAngleServo.setPosition((-gamepad1.right_stick_y + 1.0) / 2.0);

            telemetry.addData("Servo Position", launcherAngleServo.getPosition());
            telemetry.update();
        }
    }
}
