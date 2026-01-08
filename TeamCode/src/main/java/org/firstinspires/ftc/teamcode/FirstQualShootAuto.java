//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous
//public class FirstQualShootAuto extends LinearOpMode {
//    DcMotor smallLauncherWheels, mainLauncher, intake,mainLauncher2;
//    CRServo servoLaunchRight, servoLaunchLeft;
//    Boolean maxSpeed = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Boolean launchOn = true;
//        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
//        mainLauncher = hardwareMap.dcMotor.get("ml2");
////        mainLauncher2  =hardwareMap.dcMotor.get("ml2");
//        intake = hardwareMap.dcMotor.get("intake");
//        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
//        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");
//        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        mainLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        waitForStart();
//
//
//        drive.setWeightedDrivePower(
//                new Pose2d(-0.6, 0, 0)
//        );
//        sleep(800);
//        drive.setWeightedDrivePower(
//                new Pose2d(0, 0, 0)
//        );
//        mainLauncher.setPower(.825);
//        sleep(6000);
//        smallLauncherWheels.setPower(1);
//        sleep(3000);
//        servoLaunchLeft.setPower(1);
//        servoLaunchRight.setPower(1);
//        sleep(5000);
//        mainLauncher.setPower(0);
//        servoLaunchLeft.setPower(0);
//        servoLaunchRight.setPower(0);
//        mainLauncher.setPower(0);
//        smallLauncherWheels.setPower(0);
//        drive.setWeightedDrivePower(
//                new Pose2d(0, 0.5, 0)
//        );
//        sleep(700);
//        drive.setWeightedDrivePower(
//                new Pose2d(0, 0, 0)
//        );
//    }
//
//    public void shoot() {
//        servoLaunchLeft.setPower(1);
//        servoLaunchRight.setPower(1);
//    }
//}
