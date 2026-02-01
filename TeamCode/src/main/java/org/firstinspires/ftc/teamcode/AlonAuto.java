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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AlonAuto extends LinearOpMode {
    MecanumDrive drive;
    double[] targets = new double[3];
    double[] irl = new double[3];
    int shotNum = 2;
    Vector2d TARGET_POS = new Vector2d(-64, -64);
    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;
    Servo angleServo;
    final double MULT = 1.5;
    final Pose2d SHOOT_POS = new Pose2d(-17, 17, 3 * Math.PI / 4);
    final Pose2d SHOOT_POS_LONG = new Pose2d(60, 20, Math.toRadians(150));


    @Override
    public void runOpMode() throws InterruptedException {
        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");
        angleServo = hardwareMap.get(Servo.class, "aim");
        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");
        smallLauncherWheels.setDirection(DcMotorSimple.Direction.REVERSE);
        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mainLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(25, 0, 0, 23.5);
        mainLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        angleServo.setDirection(Servo.Direction.REVERSE);
        mainLauncher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        drive = new MecanumDrive(hardwareMap, new Pose2d(60, 20, Math.toRadians(180)));
        waitForStart();

        Action turn = drive.actionBuilder(new Pose2d(60, 20, Math.toRadians(180))
        ).turn(-Math.PI / 8).build();
        Action shootFirstThree = new SequentialAction(
                turn,
                new InstantAction(this::spinUpPID),
                new InstantAction(this::powerLong),
                new SleepAction(4.5),
                shootLong()
        );


        Action getFirstRow = drive.actionBuilder(
                        new Pose2d(60, 20, Math.toRadians(180) - Math.PI / 6)
                )
                .strafeToLinearHeading(new Vector2d(28, 15), Math.PI / 2)
                .strafeTo(new Vector2d(28, 68))
                .build();
        Action getSecondRow = drive.actionBuilder(
                        new Pose2d(-17, 17, 3 * Math.PI / 4)
                )
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(6, 13), Math.PI / 2)
                .strafeTo(new Vector2d(6, 69))
                .build();
        Action getThirdRow = drive.actionBuilder(
                        new Pose2d(-17, 17, 3 * Math.PI / 4)
                )
                .strafeToLinearHeading(new Vector2d(-17, 13), Math.PI / 2)
                .strafeTo(new Vector2d(-17, 69))
                .build();
        Action have = new InstantAction(() -> {
            drive.updatePoseEstimate();
            drive.localizer.update();
            AimingUtil.storedPose = new Pose2d(-drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, -drive.localizer.getPose().heading.toDouble());
        });
        Action wait = drive.actionBuilder(new Pose2d(-13, 13, 3 * Math.PI / 4))
                .waitSeconds(1)
                .build();
        Action goToLaunch1 = drive.actionBuilder(
                        new Pose2d(new Vector2d(28, 69), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-17, 17, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch2 = drive.actionBuilder(
                        new Pose2d(new Vector2d(6, 69), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-17, 17, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch3 = drive.actionBuilder(
                        new Pose2d(new Vector2d(-17, 69), Math.PI / 2)
                )
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-17, 17), 3 * Math.PI / 4)
                .build();

        Action postShootBehavior = new SequentialAction(
                idleServos(),
                idleLaunchMotors()
        );
        Action postShootBehavior0 = new SequentialAction(
                idleServos(),
                idleLaunchMotors()
        );
        Action postShootBehavior3 = new SequentialAction(
                idleServos(),
                idleLaunchMotors()
        );
        Action fullIntake = new SequentialAction(
                intake(),
                new SleepAction(3),
                idleIntake(),
                idleTransfer()
        );
        Action fullAuto = new SequentialAction(
                shootFirstThree,
                postShootBehavior0,
//                new ParallelAction(getFirstRow,intake()),
                new ParallelAction(new SequentialAction(getFirstRow, goToLaunch1), fullIntake, getToPower()),
//                new ParallelAction(goToLaunch1, getToPower()),
                shoot(),
                postShootBehavior,
//                new ParallelAction(getSecondRow,intake()),
                new ParallelAction(new SequentialAction(getSecondRow, goToLaunch2), fullIntake, getToPower()),
//                new ParallelAction(goToLaunch2, getToPower()),
                shoot(),
                postShootBehavior3,
                new InstantAction(() -> {
                    drive.updatePoseEstimate();
                    drive.localizer.update();
                    AimingUtil.storedPose = new Pose2d(-drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble());
                }),
//                new ParallelAction(getThirdRow,intake()),
                new ParallelAction(new SequentialAction(getThirdRow, goToLaunch3), fullIntake, getToPower(), new SequentialAction(new SleepAction(1), have)),
//                new ParallelAction(goToLaunch3, getToPower()),
                shoot(),
                new InstantAction(() -> {
                    drive.updatePoseEstimate();
                    drive.localizer.update();
                    AimingUtil.storedPose = new Pose2d(-drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, -drive.localizer.getPose().heading.toDouble());
                }),
                new SleepAction(3)
        );
        while (opModeIsActive()) {
        Actions.runBlocking(fullAuto);
    }
        drive.updatePoseEstimate();
        drive.localizer.update();
        AimingUtil.storedPose = new Pose2d(-drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, -drive.localizer.getPose().heading.toDouble());

//        Trajectory traj = drive.TrajectoryBuilder( 22, Math.toRadians(180)
//                .splineTo(new Vector2d(14, 22), Math.toRadians(160))
//                .waitSeconds(1)
//                .splineTo(new Vector2d(8, 52), Math.toRadians(180))
//                .waitSeconds(1)
//                .build());
    }
    public void power() {
        Pose2d pose = SHOOT_POS;
        Vector2d diff = pose.position.minus(TARGET_POS);
        double distToGoal = Math.hypot(diff.x, diff.y);
        double servoDeg = AimingUtil.DistanceToAngle(distToGoal, AimingUtil.SERVO_MIN_DEG, AimingUtil.SERVO_MAX_DEG);
        double targetRPM;
        if(shotNum == 2) {
            targetRPM = AimingUtil.DistanceToRPM(distToGoal);
        } else{
            targetRPM = AimingUtil.DistanceToRPM(distToGoal - 12);
        }
        double motorVelo = AimingUtil.getTargetVelocity(targetRPM);
        angleServo.setPosition((servoDeg/30) - 1);
        mainLauncher.setVelocity(motorVelo);
        mainLauncher2.setVelocity(motorVelo);
    }
    public void powerLong() {
        Pose2d pose = SHOOT_POS_LONG;
        Vector2d diff = pose.position.minus(TARGET_POS);
        double distToGoal = Math.hypot(diff.x, diff.y);
        double servoDeg = 55;
        double targetRPM;
        if(shotNum == 2) {
            targetRPM = AimingUtil.DistanceToRPM(distToGoal);
        } else{
            targetRPM = AimingUtil.DistanceToRPM(distToGoal);
        }
        telemetry.addData("servo angle", servoDeg);
        telemetry.addData("rpm", 60*(mainLauncher.getVelocity()/28));
        telemetry.addData("position", drive.localizer.getPose().position);
        telemetry.addData("target RPM", AimingUtil.DistanceToRPM(distToGoal));
        telemetry.update();
        double motorVelo = AimingUtil.getTargetVelocity(targetRPM);
        angleServo.setPosition((servoDeg/30) - 1);
        mainLauncher.setVelocity(motorVelo);
        mainLauncher2.setVelocity(motorVelo);
    }
    public void spinUpPID() {
        mainLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0, 20));
        mainLauncher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0, 20));
        telemetry.addData("PID 1", mainLauncher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PID 2", mainLauncher2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();
    }
    public void shootPID() {
        mainLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0, 20));
        mainLauncher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0, 20));
        telemetry.addData("PID 1", mainLauncher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PID 2", mainLauncher2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();
    }
    public Action getToPower() {
        return new SequentialAction(new InstantAction(this::spinUpPID), new InstantAction(this::power));
    }

//    private void power(int s) {
//    }


    public void launch() {
        shootPID();
        shotNum--;
        servoLaunchLeft.setPower(1);
        servoLaunchRight.setPower(1);
        smallLauncherWheels.setPower(1);
    }
    public Action shoot() {
        return new SequentialAction(
                new InstantAction(() -> {
                    Vector2d diff = SHOOT_POS.position.minus(TARGET_POS);
                    double distToGoal = Math.hypot(diff.x, diff.y);
                   targets[2-shotNum]  = AimingUtil.DistanceToRPM(distToGoal);
                   irl[2-shotNum] = 60 * (mainLauncher.getVelocity() / 28);
                }),
                new InstantAction(this::launch),
                new InstantAction(() -> {
                    Vector2d diff = SHOOT_POS.position.minus(TARGET_POS);
                    double distToGoal = Math.hypot(diff.x, diff.y);
                    telemetry.addData(
                        "motor speed (RPM)",
                        "L1: " + 60 * (mainLauncher.getVelocity() / 28) +
                                " L2: " + 60 * (mainLauncher2.getVelocity() / 28)
                );
        telemetry.addData("position", drive.localizer.getPose().position);
        telemetry.addData("heading",drive.localizer.getPose().heading.toDouble());
        telemetry.addData("target RPM", AimingUtil.DistanceToRPM(distToGoal));
        telemetry.update();
                }),
                new SleepAction(1.5)
        );
    }
    public Action shootLong() {
        return new SequentialAction(
                new InstantAction(() -> {
                    Vector2d diff = SHOOT_POS_LONG.position.minus(TARGET_POS);
                    double distToGoal = Math.hypot(diff.x, diff.y);
                    targets[2-shotNum]  = AimingUtil.DistanceToRPM(distToGoal);
                    irl[2-shotNum] = 60 * (mainLauncher.getVelocity() / 28);
                }),
                new InstantAction(this::launch),
                new InstantAction(() -> {
                    Vector2d diff = SHOOT_POS_LONG.position.minus(TARGET_POS);
                    double distToGoal = Math.hypot(diff.x, diff.y);
                    telemetry.addData(
                            "motor speed (RPM)",
                            "L1: " + 60 * (mainLauncher.getVelocity() / 28) +
                                    " L2: " + 60 * (mainLauncher2.getVelocity() / 28)
                    );
                    telemetry.addData("distance", distToGoal);
                    telemetry.addData("position", drive.localizer.getPose().position);
                    telemetry.addData("heading",drive.localizer.getPose().heading.toDouble());
                    telemetry.addData("target RPM", AimingUtil.DistanceToRPM(distToGoal + 36));
                    telemetry.update();
                }),
                new SleepAction(2)
        );
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
            intake.setPower(1);
            smallLauncherWheels.setPower(1);
    }
    public  void killTransfer() {
        smallLauncherWheels.setPower(1);
    }
    public Action idleTransfer() {
        return new InstantAction(this::killTransfer);
    }
    public void killIntake() {
        intake.setPower(1);
    }
    public Action idleIntake() {
        return new InstantAction(this::killIntake);
    }
}
