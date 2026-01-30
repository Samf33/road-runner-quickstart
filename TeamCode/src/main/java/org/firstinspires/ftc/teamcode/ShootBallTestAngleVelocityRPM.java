package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class ShootBallTestAngleVelocityRPM extends LinearOpMode {

    final double SERVO_MIN_DEG = 30.0;
    final double SERVO_MAX_DEG = 60.0;

    double angleInput = 0;
    double powerInput = 0.5;

    boolean shootToggle = true;
    boolean upLatch, downLatch, leftLatch, rightLatch, xLatch;

    Servo angleServo;
    DcMotorEx launcherA, launcherB;
    DcMotor smallLauncherWheels;
    CRServo feederLeft, feederRight;
    MecanumDrive drive;
    final Vector2d TARGET_POS = new Vector2d(57.98275605748032, 57.98275605748032);

    @Override
    public void runOpMode() {
        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        angleServo = hardwareMap.get(Servo.class, "aim");
        launcherA = hardwareMap.get(DcMotorEx.class, "ml");
        launcherB = hardwareMap.get(DcMotorEx.class, "ml2");
        feederLeft = hardwareMap.get(CRServo.class, "slLeft");
        feederRight = hardwareMap.get(CRServo.class, "slRight");
        launcherA.setDirection(DcMotorSimple.Direction.REVERSE);
        smallLauncherWheels.setDirection(DcMotorSimple.Direction.REVERSE);
        feederLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(SERVO_MIN_DEG / 360);
//        launcherA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 0, 2, 12));
//        launcherB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 0, 2, 12));
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            gamepad1.right_stick_x
                    )
            );
            drive.localizer.update();
            drive.updatePoseEstimate();
            if (gamepad1.dpad_up) {
                angleInput += 0.005;
                upLatch = true;
            } else if (!gamepad1.dpad_up) upLatch = false;

            if (gamepad1.dpad_down) {
                angleInput -= 0.005;
                downLatch = true;
            } else if (!gamepad1.dpad_down) downLatch = false;

            if (gamepad1.dpad_right && !rightLatch) {
                powerInput += 0.025;
                rightLatch = true;
            } else if (!gamepad1.dpad_right) rightLatch = false;

            if (gamepad1.dpad_left && !leftLatch) {
                powerInput -= 0.025;
                leftLatch = true;
            } else if (!gamepad1.dpad_left) leftLatch = false;

            if (gamepad1.x && !xLatch) {
                shootToggle = !shootToggle;
                xLatch = true;
            } else if (!gamepad1.x) xLatch = false;

            angleInput = Math.max(0.0, Math.min(1.0, angleInput));
            powerInput = Math.max(0.0, Math.min(1.0, powerInput));

            double servoDeg = SERVO_MIN_DEG + angleInput * (SERVO_MAX_DEG - SERVO_MIN_DEG);
            angleServo.setPosition((servoDeg/30) - 1);

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
                smallLauncherWheels.setPower(1);
            } else {
                feederLeft.setPower(-0.3);
                feederRight.setPower(-0.3);
                smallLauncherWheels.setPower(0);
            }

            Pose2d pose = drive.localizer.getPose();
            Vector2d diff = pose.position.minus(TARGET_POS);

            telemetry.addData("Angle (deg)", servoDeg);
            telemetry.addData("pos", drive.localizer.getPose().position);
            telemetry.addData("Power", powerInput);
            telemetry.addData("RPM A", 60 * (launcherA.getVelocity() / 28.0));
            telemetry.addData("RPM B", 60 * (launcherB.getVelocity() / 28.0));
            telemetry.addData("Distance (in)", Math.hypot(diff.x, diff.y));
            telemetry.update();
        }
    }
}
