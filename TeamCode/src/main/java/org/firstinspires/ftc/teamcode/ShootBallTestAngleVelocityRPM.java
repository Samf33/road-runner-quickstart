package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class ShootBallTestAngleVelocityRPM extends LinearOpMode {

    final double SERVO_MIN_DEG = 30.0;
    final double SERVO_MAX_DEG = 60.0;

    double angleInput = 0.5;
    double powerInput = 0.0;

    boolean shootToggle = false;
    boolean upLatch, downLatch, leftLatch, rightLatch, xLatch;

    Servo angleServo;
    DcMotorEx launcherA, launcherB;
    CRServo feederLeft, feederRight;

    final Vector2d TARGET_POS = new Vector2d(147.276200386, 147.276200386);

    @Override
    public void runOpMode() {

        angleServo = hardwareMap.get(Servo.class, "launcherAngleServo");
        launcherA = hardwareMap.get(DcMotorEx.class, "ml");
        launcherB = hardwareMap.get(DcMotorEx.class, "ml2");
        feederLeft = hardwareMap.get(CRServo.class, "slLeft");
        feederRight = hardwareMap.get(CRServo.class, "slRight");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        angleServo.setPosition(SERVO_MIN_DEG / 180.0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !upLatch) {
                angleInput += 0.05;
                upLatch = true;
            } else if (!gamepad1.dpad_up) upLatch = false;

            if (gamepad1.dpad_down && !downLatch) {
                angleInput -= 0.05;
                downLatch = true;
            } else if (!gamepad1.dpad_down) downLatch = false;

            if (gamepad1.dpad_right && !rightLatch) {
                powerInput += 0.05;
                rightLatch = true;
            } else if (!gamepad1.dpad_right) rightLatch = false;

            if (gamepad1.dpad_left && !leftLatch) {
                powerInput -= 0.05;
                leftLatch = true;
            } else if (!gamepad1.dpad_left) leftLatch = false;

            if (gamepad1.x && !xLatch) {
                shootToggle = !shootToggle;
                xLatch = true;
            } else if (!gamepad1.x) xLatch = false;

            angleInput = Math.max(0.0, Math.min(1.0, angleInput));
            powerInput = Math.max(0.0, Math.min(1.0, powerInput));

            double servoDeg = SERVO_MIN_DEG + angleInput * (SERVO_MAX_DEG - SERVO_MIN_DEG);
            angleServo.setPosition(servoDeg / 180.0);

            if (shootToggle) {
                launcherA.setPower(powerInput);
                launcherB.setPower(powerInput);
            } else {
                launcherA.setPower(0);
                launcherB.setPower(0);
            }

            if (gamepad1.right_trigger > 0.3) {
                feederLeft.setPower(1);
                feederRight.setPower(1);
            } else {
                feederLeft.setPower(-0.3);
                feederRight.setPower(-0.3);
            }

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            Vector2d diff = pose.position.minus(TARGET_POS);

            telemetry.addData("Angle (deg)", servoDeg);
            telemetry.addData("Power", powerInput);
            telemetry.addData("RPM A", 60 * (launcherA.getVelocity() / 28.0));
            telemetry.addData("RPM B", 60 * (launcherB.getVelocity() / 28.0));
            telemetry.addData("Distance (cm)", Math.hypot(diff.x, diff.y));
            telemetry.update();
        }
    }
}
