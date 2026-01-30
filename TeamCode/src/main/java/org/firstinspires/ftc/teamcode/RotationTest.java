package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class RotationTest extends LinearOpMode {

    MecanumDrive drive;

    double targetX = 0;
    double targetY = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));

        waitForStart();

        while (!isStopRequested()) {
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            double dx = targetX - pose.position.x;
            double dy = targetY - pose.position.y;

            double newHeading = Math.atan2(dy, dx);
            double error = newHeading - pose.heading.toDouble() % 360;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), error));

            telemetry.update();
        }
    }
}
