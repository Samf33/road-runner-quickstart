package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    static RoadRunnerBotEntity myBot;
    static int track = 0;
    final double MULT = 1.5;
    final Pose2d SHOOT_POS = new Pose2d(-13, 13, 3 * Math.PI / 4);


    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(new Constraints(50, 50, Math.PI, Math.PI, 17.5))
                .build();


        Action getFirstRow = myBot.getDrive().actionBuilder(
                        new Pose2d(60, 20, Math.toRadians(180))
                )
                .waitSeconds(5)
                .strafeTo(new Vector2d(36, 15))
                .waitSeconds(1)
                .strafeTo(new Vector2d(36, 60))

                .build();
        Action getSecondRow = myBot.getDrive().actionBuilder(
                        new Pose2d(-13, 13, 3 * Math.PI / 4)
                )
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(12, 13), Math.PI / 2)
                .strafeTo(new Vector2d(12, 60))
                .build();
        Action getThirdRow = myBot.getDrive().actionBuilder(
                        new Pose2d(-13, 13, 3 * Math.PI / 4)
                )
                .strafeToLinearHeading(new Vector2d(-15, 13), Math.PI / 2)
                .strafeTo(new Vector2d(-15, 60))
                .build();

        Action wait = myBot.getDrive().actionBuilder(new Pose2d(-13, 13, 3 * Math.PI / 4))
                .waitSeconds(1)
                .build();
        Action goToLaunch1 = myBot.getDrive().actionBuilder(
                        new Pose2d(new Vector2d(36, 60), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-13, 13, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch2 = myBot.getDrive().actionBuilder(
                        new Pose2d(new Vector2d(12, 60), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-13, 13, 3 * Math.PI / 4), new Rotation2d(-Math.PI / 4, -Math.PI / 4))
                .build();
        Action goToLaunch3 = myBot.getDrive().actionBuilder(
                        new Pose2d(new Vector2d(-15, 60), Math.PI / 2)
                )
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-13, 13), 3 * Math.PI / 4)
                .build();
        Action fullShoot = new SequentialAction(
//                shoot(),
                new SleepAction(1.5)
    //                idleServos(),
    //                idleLaunchMotors()
        );
        Action fullIntake = new SequentialAction(
                new SleepAction(1),
//                intake(),
                new SleepAction(3)
//                idleIntake(),
//                idleTransfer()
        );
        Action fullAuto = new SequentialAction(
//                new ParallelAction(getFirstRow,intake()),
                new ParallelAction(new SequentialAction(getFirstRow, goToLaunch1), fullIntake),
//                new ParallelAction(goToLaunch1, getToPower()),
                fullShoot,
//                new ParallelAction(getSecondRow,intake()),
                new ParallelAction(new SequentialAction(getSecondRow, goToLaunch2), fullIntake),
//                new ParallelAction(goToLaunch2, getToPower()),
                fullShoot,
//                new ParallelAction(getThirdRow,intake()),
                new ParallelAction(new SequentialAction(getThirdRow, goToLaunch3), fullIntake),
//                new ParallelAction(goToLaunch3, getToPower()),
                fullShoot
        );
        myBot.runAction(fullAuto);
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


    //        Trajectory traj = mybot.getDrive().TrajectoryBuilder( 22, Math.toRadians(180)
//                .splineTo(new Vector2d(14, 22), Math.toRadians(160))
//                .waitSeconds(1)
//                .splineTo(new Vector2d(8, 52), Math.toRadians(180))
//                .waitSeconds(1)
//                .build());
    public void power() {
//        Pose2d pose = SHOOT_POS;
//        Vector2d diff = pose.position.minus(AimingUtil.TARGET_POS);
//        double distToGoal = Math.hypot(diff.x, diff.y);
//        double servoDeg = AimingUtil.DistanceToAngle(distToGoal, AimingUtil.SERVO_MIN_DEG, AimingUtil.SERVO_MAX_DEG);
//        double targetRPM = AimingUtil.DistanceToRPM(distToGoal);
//        double motorVelo = AimingUtil.getTargetVelocity(targetRPM);
//        angleServo.setPosition((servoDeg / 30) - 1);
//        mainLauncher.setVelocity(motorVelo);
//        mainLauncher2.setVelocity(motorVelo);
        System.out.println("hi");
    }

    public Action getToPower() {
        return new InstantAction(this::power);
    }

    public void launch() {
//        servoLaunchLeft.setPower(1);
//        servoLaunchRight.setPower(1);
//        smallLauncherWheels.setPower(1);
    }

    public Action shoot() {
        return new InstantAction(this::launch);
    }

    public Action intake() {
        return new InstantAction(this::in);
    }

    public void killLaunch() {
//        mainLauncher.setPower(.25);
//        mainLauncher2.setPower(.25);
    }

    public Action idleLaunchMotors() {
        return new InstantAction(this::killLaunch);
    }

    public void killServos() {
//        servoLaunchLeft.setPower(-.3);
//        servoLaunchRight.setPower(-.3);
    }

    public Action idleServos() {
        return new InstantAction(this::killServos);
    }

    public void in() {
//        intake.setPower(1);
//        smallLauncherWheels.setPower(1);
    }

    public void killTransfer() {
//        smallLauncherWheels.setPower(1);
    }

    public Action idleTransfer() {
        return new InstantAction(this::killTransfer);
    }

    public void killIntake() {
//        intake.setPower(0);
    }

    public Action idleIntake() {
//        return new InstantAction(this::killIntake);
        return null;
    }
}


