package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(new Constraints(50, 50, Math.PI, Math.PI, 17.5))
                .build();


        Action getFirstRow = myBot.getDrive().actionBuilder(
                        new Pose2d(60, 20, Math.toRadians(180))
                )
                .splineTo(new Vector2d(36, 15), Math.PI / 2)
                .lineToY(45)

                .build();
        Action getSecondRow = myBot.getDrive().actionBuilder(
                        new Pose2d(-10, 10, 3*Math.PI/4)
                )
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(13, 10) , Math.PI / 2)
                .strafeTo(new Vector2d(13, 45))
                .build();
        Action getThirdRow = myBot.getDrive().actionBuilder(
                        new Pose2d(-10, 10, 3*Math.PI/4)
                )
                .strafeToLinearHeading(new Vector2d(-10, 45), Math.PI/2)
                .build();

        Action wait = myBot.getDrive().actionBuilder(new Pose2d(-10, 10, 3*Math.PI/4))
                .waitSeconds(1)
                .build();

        Action goToLaunch1 = myBot.getDrive().actionBuilder(
                new Pose2d(new Vector2d(36, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .build();
        Action goToLaunch2 = myBot.getDrive().actionBuilder(
                        new Pose2d(new Vector2d(15, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .build();
        Action goToLaunch3 = myBot.getDrive().actionBuilder(
                        new Pose2d(new Vector2d(-10, 45), Math.PI / 2)
                )
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-10, 10), 3*Math.PI/4)
                .build();

        Action fullAuto = new SequentialAction(
                getFirstRow,
                setPos(36, 45, Math.PI / 2),
                goToLaunch1,
                wait,
                getSecondRow,
                setPos(15, 45, Math.PI / 2),
                goToLaunch2,
                wait,
                getThirdRow,
                goToLaunch3



        );

        myBot.runAction(fullAuto);
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    public static Action setPos(double x, double y, double head) {
        return new InstantAction(() ->
                track += 1
        );
    }
}