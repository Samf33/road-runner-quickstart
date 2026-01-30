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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "drive")
public class FirstQualNonAuto extends LinearOpMode {

    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;
    Servo angleServo;
    double angleInput = 0;
    final double SERVO_MIN_DEG = 30.0;
    final double SERVO_MAX_DEG = 60.0;
    final Vector2d TARGET_POS = new Vector2d(57.98275605748032, 57.98275605748032);

    boolean maxSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean launchOn = true;
        int mode = 0;

        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");
        angleServo = hardwareMap.get(Servo.class, "aim");
        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");
        mainLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        smallLauncherWheels.setDirection(DcMotorSimple.Direction.REVERSE);
        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        mainLauncher2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(0, 0, 0, 11.7);
        mainLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        angleServo.setDirection(Servo.Direction.REVERSE);
        mainLauncher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();


        while (!isStopRequested()) {
            boolean isAiming = gamepad1.left_bumper;

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            isAiming ? AimingUtil.getVelocityToAim(57.98275605748032, 57.98275605748032, drive.localizer.getPose()) : -gamepad1.right_stick_x
                    )
            );
//            double servoDeg = SERVO_MIN_DEG + angleInput * (SERVO_MAX_DEG - SERVO_MIN_DEG);
            Pose2d pose = drive.localizer.getPose();
            Vector2d diff = pose.position.minus(TARGET_POS);
            double distToGoal = Math.hypot(diff.x, diff.y);
            double servoDeg = AimingUtil.DistanceToAngle(distToGoal, SERVO_MIN_DEG, SERVO_MAX_DEG);
            double targetRPM = AimingUtil.DistanceToRPM(distToGoal);
            double motorVelo = AimingUtil.getTargetVelocity(targetRPM);
            angleServo.setPosition((servoDeg/30) - 1);
            telemetry.addData(
                    "Servo Speeds",
                    "left: " + servoLaunchLeft.getPower() +
                            ", right: " + servoLaunchRight.getPower()
            );

            telemetry.addData(
                    "mode",
                    mode % 3 + "    0 = launch1 | 1 = launch2 | 2 = both"
            );
            telemetry.addData(
                    "motor speed (RPM)",
                    "L1: " + 60 * (mainLauncher.getVelocity() / 28) +
                            " L2: " + 60 * (mainLauncher2.getVelocity() / 28)
            );
            telemetry.addData("position", drive.localizer.getPose().position);
            telemetry.addData("heading",drive.localizer.getPose().heading.toDouble());
            telemetry.addData("target RPM", targetRPM);

            telemetry.update();

            if (gamepad1.right_trigger >= 0.3) {
                in();
            } else {
                intake.setPower(0);
                smallLauncherWheels.setPower(0);
            }

            if (gamepad1.left_trigger >= 0.3) {
                shoot();
            } else {
                servoLaunchLeft.setPower(-.3);
                servoLaunchRight.setPower(-.3);
            }



            if (gamepad1.a) {
                launchOn = !launchOn;
            }

            if (launchOn) {
                    mainLauncher.setVelocity(motorVelo);
                    mainLauncher2.setVelocity(motorVelo);
            } else {
                mainLauncher.setPower(0);
                mainLauncher2.setPower(0);
            }
            drive.updatePoseEstimate();
            drive.localizer.update();
        }

    }

    // === HELPERS ===

    public void shoot() {
        servoLaunchLeft.setPower(1);
        servoLaunchRight.setPower(1);
    }

    public void in() {
        if (!gamepad1.x) {
            intake.setPower(gamepad1.right_trigger * 0.8);
        } else {
            intake.setPower(-0.75);
        }
        smallLauncherWheels.setPower(0.9);
    }
}
