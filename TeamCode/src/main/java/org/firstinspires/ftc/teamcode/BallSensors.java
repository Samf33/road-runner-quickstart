package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(group = "drive")
public class BallSensors extends LinearOpMode {

    final double SEARCH_SPEED = 0.2;
    final double FOUND_ROTATE_SPEED = 0.4;
    final double FOUND_MOVE_SPEED = 0.8;

    private Limelight3A limelight;
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        waitForStart();

        limelight.pipelineSwitch(1); // Go to 0th pipeline

        while (!isStopRequested()) {
            telemetry.addData("limelight working", limelight.isRunning());
            LLResult ball = limelight.getLatestResult();

            if (ball != null && ball.isValid()) { //WE FOUND A BALL
                ballLoop(ball.getTx());
                telemetry.addData("Status", "Going to ball");
            } else { // NO BALL
                noBallLoop();
                telemetry.addData("Status", "Searching for ball");
            }

            telemetry.update();
        }
    }

    public void ballLoop(double targetXOffset) {
        boolean goRight = targetXOffset <= 0;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                FOUND_MOVE_SPEED,  // forward
                                0.0                // strafe
                        ),
                        goRight ? FOUND_ROTATE_SPEED : -FOUND_ROTATE_SPEED
                )
        );
    }



    public void noBallLoop() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0.0, 0.0),
                        SEARCH_SPEED
                )
        );
    }


}
