//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous
//public class FirstQualDummyAutoForward extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        waitForStart();
//
//        drive.setWeightedDrivePower(
//            new Pose2d(0.5, 0, 0)
//        );
//        sleep(700);
//        drive.setWeightedDrivePower(
//                new Pose2d(0, 0, 0)
//        );
//    }
//}
