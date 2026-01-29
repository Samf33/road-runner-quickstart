package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(group = "drive")
public class DynamicAuto extends Maccabot {

    // CONSTANTS
    final double SEARCH_SPEED = 0.2;
    final double FOUND_ROTATE_SPEED = 0.4;
    final double FOUND_MOVE_SPEED = 0.8;
    final double CORRECT_ROTATION_SPEED = 0.2;
    final double LOOK_THRESHOLD = 5;
    final int EXTRA_BALL_CATCHING_MOVEMENT_TIME = 700;

    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        int numBalls = 0;

        while (!isStopRequested()) {
            if (goAndGetABall()) {
                numBalls++;
            }

            if (numBalls >= 3) {
                //shoot
            }
        }
    }

    private boolean goAndGetABall() { // returns true if it got a ball
        double[] ball = getBall();
        while ((ball = getBall()) == null) { // SEARCHES UNTIL IT FINDS A BALL
            setDrive(0, 0, SEARCH_SPEED);
            idle();
        }

        while (faceTowardsBall(ball) < LOOK_THRESHOLD) {
            ball = getBall();
            if (ball == null) {
                return false;
            }


        }


        setIntake(true);
        while ((ball = getBall()) != null) {
            boolean goRight = ball[0] <= 0;

            updatePoseEstimate();
            Pose2d position = getPosition();
            double nextPositionX = position.position.x + position.heading.vec().x * FOUND_MOVE_SPEED * 5;

            if (position.position.x * nextPositionX <= 0) {
                goTo(0, 0, 0);
                return false;
            }

            setDrive(FOUND_MOVE_SPEED, 0, goRight ? CORRECT_ROTATION_SPEED : -CORRECT_ROTATION_SPEED);
            idle();
        }

        setDrive(FOUND_MOVE_SPEED, 0, 0);
        sleep(EXTRA_BALL_CATCHING_MOVEMENT_TIME);
        stopDrive();
        setIntake(false);

        return true;
    }

    private double[] getBall() {
        LLResult greenBall = getLimelightResult(0);
        boolean greenBallIsValid = greenBall != null && greenBall.isValid();

        LLResult purpleBall = getLimelightResult(1);
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

    public double faceTowardsBall(double[] ball) { // returns how off it is from the ball
        boolean goRight = ball[0] <= 0;
        setDrive(0,0, goRight ? FOUND_ROTATE_SPEED : -FOUND_ROTATE_SPEED);
        return Math.abs(ball[0]);
    }
}
