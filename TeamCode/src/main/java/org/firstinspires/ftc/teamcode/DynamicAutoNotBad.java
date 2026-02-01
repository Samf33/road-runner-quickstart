package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "drive")
public class DynamicAutoNotBad extends LinearOpMode {
    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;
    Servo angleServo;
    Servo rgbLight;
    private Limelight3A limelight;
    private MecanumDrive drive;

    final double ROTATE_SPEED = 50;
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, AimingUtil.storedPose == null ? new Pose2d(0,0,0) : AimingUtil.storedPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(Ball.PURPLE.ordinal());

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
        mainLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, AimingUtil.LAUNCH_MOTOR_PID_COEFFICIENTS);
        angleServo.setDirection(Servo.Direction.REVERSE);
        mainLauncher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, AimingUtil.LAUNCH_MOTOR_PID_COEFFICIENTS);
        rgbLight = hardwareMap.get(Servo.class, "rgb");

        waitForStart();

        boolean gettingBall = false;
        int balls = 0;

        while (!isStopRequested()) {
            LLResult ball = limelight.getLatestResult();
            boolean valid = (ball != null) && (ball.isValid());
            double angularVelocity = 0;
            double forwardVelocity = 0;
            double x = 0, y = 0;

            if (valid) {
                gettingBall = true;
                x = ball.getTx();
                y = ball.getTy();

                angularVelocity = x * ROTATE_SPEED;
                forwardVelocity = 0.8;

                intake(true);
            } else {
                if (gettingBall) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            forwardVelocity, 0.0
                                    ),
                                    0
                            )
                    );
                }
                angularVelocity = 0.3;
                intake(false);
            }


            telemetry.addData("valid", valid);
            telemetry.addData("x", x);
            telemetry.addData("vel", angularVelocity);

            telemetry.update();
            
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    forwardVelocity, 0.0
                            ),
                            angularVelocity
                    )
            );
        }
    }

    public void intake(boolean on) {
        if (on) {
            intake.setPower(gamepad1.right_trigger * 0.8);
        } else {
            intake.setPower(-0.75);
        }
        smallLauncherWheels.setPower(0.9);
    }
}
