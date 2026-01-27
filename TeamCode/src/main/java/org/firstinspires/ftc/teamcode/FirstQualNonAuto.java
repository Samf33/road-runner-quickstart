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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "drive")
public class FirstQualNonAuto extends LinearOpMode {

    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;

    boolean maxSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean launchOn = true;
        int mode = 0;

        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");

        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");

        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mainLauncher2.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();

        while (!isStopRequested()) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    )
            );

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
                    "motor speed (ticks/sec)",
                    "L1: " + mainLauncher.getVelocity() +
                            " L2: " + mainLauncher2.getVelocity()
            );

            telemetry.addData(
                    "motor speed (RPS)",
                    "L1: " + mainLauncher.getVelocity() / 28 +
                            " L2: " + mainLauncher2.getVelocity() / 28
            );

            telemetry.addData(
                    "motor speed (RPM)",
                    "L1: " + 60 * (mainLauncher.getVelocity() / 28) +
                            " L2: " + 60 * (mainLauncher2.getVelocity() / 28)
            );

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
                servoLaunchLeft.setPower(0);
                servoLaunchRight.setPower(0);
            }

            if (gamepad1.x) {
                mode += 1;
                sleep(200); // debounce
            }

            if (gamepad1.a) {
                launchOn = !launchOn;
                sleep(200); // debounce
            }

            if (gamepad1.dpad_right) {
                servoLaunchRight.setPower(1);
            }
            if (gamepad1.dpad_left) {
                servoLaunchLeft.setPower(1);
            }
            if (gamepad1.dpad_up) {
                mainLauncher.setPower(0.9);
            }

            if (launchOn) {
                if (mode % 3 == 0) {
                    mainLauncher.setPower(1);
                    mainLauncher2.setPower(0);
                } else if (mode % 3 == 1) {
                    mainLauncher2.setPower(1);
                    mainLauncher.setPower(0);
                } else {
                    mainLauncher.setPower(1);
                    mainLauncher2.setPower(1);
                }
            } else {
                mainLauncher.setPower(0);
                mainLauncher2.setPower(0);
            }
            drive.updatePoseEstimate();
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
