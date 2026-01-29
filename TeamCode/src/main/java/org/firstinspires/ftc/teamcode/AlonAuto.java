package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Action.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.google.gson.internal.bind.SqlDateTypeAdapter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AlonAuto extends LinearOpMode {
    MecanumDrive drive;
    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");

        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");

        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mainLauncher2.setDirection(DcMotorSimple.Direction.REVERSE);
        drive = new MecanumDrive(hardwareMap, new Pose2d(60, 20, Math.toRadians(180)));

        waitForStart();
        Action getFirstRow = drive.actionBuilder(
                        new Pose2d(60, 20, Math.toRadians(180))
                )
                .splineTo(new Vector2d(36, 15), Math.PI / 2)
                .lineToY(45)

                .build();
        Action getSecondRow = drive.actionBuilder(
                        new Pose2d(-10, 10, 3 * Math.PI / 4)
                )
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(13, 10), Math.PI / 2)
                .strafeTo(new Vector2d(13, 45))
                .build();
        Action getThirdRow = drive.actionBuilder(
                        new Pose2d(-10, 10, 3 * Math.PI / 4)
                )
                .strafeToLinearHeading(new Vector2d(-10, 45), Math.PI / 2)
                .build();

        Action wait = drive.actionBuilder(new Pose2d(-10, 10, 3 * Math.PI / 4))
                .waitSeconds(1)
                .build();
        Action goToLaunch1 = drive.actionBuilder(
                        new Pose2d(new Vector2d(36, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch2 = drive.actionBuilder(
                        new Pose2d(new Vector2d(15, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch3 = drive.actionBuilder(
                        new Pose2d(new Vector2d(-10, 45), Math.PI / 2)
                )
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-10, 10), 3 * Math.PI / 4)
                .build();
        Action fullShoot = new SequentialAction(
                shoot(),
                new SleepAction(1.5),
                idleServos(),
                idleLaunchMotors()
        );
        Action fullIntake = new SequentialAction(
                intake(),
                new SleepAction(1.5),
                idleIntake(),
                idleTransfer()
        );
        Action fullAuto = new SequentialAction(
//                new ParallelAction(getFirstRow,intake()),
                new ParallelAction(new SequentialAction(getFirstRow, goToLaunch1),fullIntake, getToPower()),
//                new ParallelAction(goToLaunch1, getToPower()),
                fullShoot,
//                new ParallelAction(getSecondRow,intake()),
                new ParallelAction(new SequentialAction(getSecondRow, goToLaunch2),fullIntake, getToPower()),
//                new ParallelAction(goToLaunch2, getToPower()),
                fullShoot,
//                new ParallelAction(getThirdRow,intake()),
                new ParallelAction(new SequentialAction(getThirdRow, goToLaunch3),fullIntake, getToPower()),
//                new ParallelAction(goToLaunch3, getToPower()),
                fullShoot
        );

        Actions.runBlocking(fullAuto);


//        Trajectory traj = drive.TrajectoryBuilder( 22, Math.toRadians(180)
//                .splineTo(new Vector2d(14, 22), Math.toRadians(160))
//                .waitSeconds(1)
//                .splineTo(new Vector2d(8, 52), Math.toRadians(180))
//                .waitSeconds(1)
//                .build());
    }
    public void power() {
        mainLauncher.setPower(1);
        mainLauncher2.setPower(1);
    }
    public Action getToPower() {
        return new InstantAction(this::power);
    }
    public void launch() {
        servoLaunchLeft.setPower(1);
        servoLaunchRight.setPower(1);
        smallLauncherWheels.setPower(1);
    }
    public Action shoot() {
        return new InstantAction(this::launch);
    }
    public Action intake() {
        return new InstantAction(this::in);
    }
    public void killLaunch() {
        mainLauncher.setPower(.25);
        mainLauncher2.setPower(.25);
    }
    public Action idleLaunchMotors(){
        return new InstantAction(this::killLaunch);
    }
    public void killServos() {
        servoLaunchLeft.setPower(-.3);
        servoLaunchRight.setPower(-.3);
    }
    public Action idleServos() {
        return new InstantAction(this::killServos);
    }
    public void in() {
            intake.setPower(0.8);
            smallLauncherWheels.setPower(0.5);
    }
    public  void killTransfer() {
        smallLauncherWheels.setPower(0);
    }
    public Action idleTransfer() {
        return new InstantAction(this::killTransfer);
    }
    public void killIntake() {
        intake.setPower(0);
    }
    public Action idleIntake() {
        return new InstantAction(this::killIntake);
    }
}
