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

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));

        waitForStart();

        while (!isStopRequested()) {
            Pose2d pose = drive.localizer.getPose();
            double targetHeading = Math.atan2(-pose.position.y, -pose.position.x);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), pose.heading.real - targetHeading));

            drive.updatePoseEstimate();

            telemetry.update();
        }
    }
}
