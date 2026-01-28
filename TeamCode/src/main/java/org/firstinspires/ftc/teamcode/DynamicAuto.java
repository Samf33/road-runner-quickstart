package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Vector;


@Autonomous(group = "drive")
public class DynamicAuto extends LinearOpMode {

    // CONSTANTS
    final double SEARCH_SPEED = 0.2;
    final double FOUND_ROTATE_SPEED = 0.4;
    final double FOUND_MOVE_SPEED = 0.8;
    final double CORRECT_ROTATION_SPEED = 0.2;
    final double LOOK_THREASHHOLD = 5;
    final int EXTRA_BALL_CATCHING_MOVEMENT_TIME = 700;

    // ROBOT INTERFACE
    private Limelight3A limelight;
    private MecanumDrive drive;
    private DcMotor intake;



    @Override
    public void runOpMode() {
        initLimelight();
        initDriveAndMotors();


        while (true) {
            goAndGetABall();
        }
    }

    private void goAndGetABall() {
        double[] ball = getBall();
        while ((ball = getBall()) == null) { // SEARCHES UNTIL IT FINDS A BALL
            search();
            idle();
        }

        while (faceTowardsBall(ball) < LOOK_THREASHHOLD) {
            ball = getBall();
            if (ball == null) {
                return;
            }
        }


        intake.setPower(-0.75);
        while ((ball = getBall()) != null) {
            grabBall(ball);
            idle();
        }

        goForward();
        sleep(EXTRA_BALL_CATCHING_MOVEMENT_TIME);
        stopMoving();
        intake.setPower(0);
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
    }

    private void initDriveAndMotors() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        waitForStart();
        intake = hardwareMap.dcMotor.get("intake");
    }

    private double[] getBall() {
        limelight.pipelineSwitch(0);
        LLResult greenBall = limelight.getLatestResult();
        boolean greenBallIsValid = greenBall != null && greenBall.isValid();

        limelight.pipelineSwitch(1);
        LLResult purpleBall = limelight.getLatestResult();
        boolean purpleBallIsValid = purpleBall != null && purpleBall.isValid();

        double[] ball = new double[2];

        if (!greenBallIsValid && !purpleBallIsValid) { // NO VALID BALLS
            ball = null;
        } else if (greenBallIsValid && !purpleBallIsValid) { // ONLY GREEN BALLS
            ball[0] = greenBall.getTx();
            ball[1] = greenBall.getTy();
        } else if (!greenBallIsValid) { // ONLY PURPLE BALLS
            ball[0] = purpleBall.getTx();
            ball[1] = purpleBall.getTy();
        } else { // BOTH WORKING BALLS
            if (greenBall.getTy() < purpleBall.getTy()) {
                ball[0] = greenBall.getTx();
                ball[1] = greenBall.getTy();
            } else {
                ball[0] = purpleBall.getTx();
                ball[1] = purpleBall.getTy();
            }
        }

        return  ball;
    }

    public void search() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0.0, 0.0),
                        SEARCH_SPEED
                )
        );
    }

    public double faceTowardsBall(double[] ball) { // returns how off it is from the ball
        boolean goRight = ball[0] <= 0;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                0.0,  // forward
                                0.0                // strafe
                        ),
                        goRight ? FOUND_ROTATE_SPEED : -FOUND_ROTATE_SPEED
                )
        );

        return Math.abs(ball[0]);
    }

    public void grabBall(double[] ball) {
        boolean goRight = ball[0] <= 0;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                FOUND_MOVE_SPEED,  // forward
                                0.0                // strafe
                        ),
                        goRight ? CORRECT_ROTATION_SPEED : -CORRECT_ROTATION_SPEED
                )
        );
    }

    public void goForward() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                FOUND_MOVE_SPEED,  // forward
                                0.0                // strafe
                        ),
                        0.0
                )
        );
    }

    public void stopMoving() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                0.0,  // forward
                                0.0                // strafe
                        ),
                        0.0
                )
        );
    }
}
