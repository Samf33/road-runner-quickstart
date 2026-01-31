package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "drive")
public class DynamicAutoNotBad extends LinearOpMode {

    private Limelight3A limelight;
    private MecanumDrive drive;

    final double ROTATE_SPEED = 0.05;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, AimingUtil.storedPose == null ? new Pose2d(0,0,0) : AimingUtil.storedPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(Ball.PURPLE.ordinal());

        waitForStart();

        while (!isStopRequested()) {
            LLResult ball = limelight.getLatestResult();
            boolean valid = (ball != null) && (ball.isValid());
            double angularVelocity = 0;

            if (valid) {
                double x = ball.getTx(), y = ball.getTy();
                angularVelocity = x * ROTATE_SPEED;
            } else {

            }
            
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    0.0, 0.0
                            ),
                            angularVelocity
                    )
            );
        }
    }
}
