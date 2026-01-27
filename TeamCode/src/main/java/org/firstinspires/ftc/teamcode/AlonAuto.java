package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Action.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.google.gson.internal.bind.SqlDateTypeAdapter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AlonAuto extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));

        waitForStart();
        Action getFirstRow = drive.actionBuilder(
                        new Pose2d(60, 20, Math.toRadians(180))
                )
                .splineTo(new Vector2d(36, 15), Math.PI / 2)
                .lineToY(45)

                .build();
        Action getSecondRow = drive.actionBuilder(
                        new Pose2d(-10, 10, 3*Math.PI/4)
                )
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(13, 10) , Math.PI / 2)
                .strafeTo(new Vector2d(13, 45))
                .build();
        Action getThirdRow = drive.actionBuilder(
                        new Pose2d(-10, 10, 3*Math.PI/4)
                )
                .strafeToLinearHeading(new Vector2d(-10, 45), Math.PI/2)
                .build();

        Action wait = drive.actionBuilder(new Pose2d(-10, 10, 3*Math.PI/4))
                .waitSeconds(1)
                .build();

        Action goToLaunch1 = drive.actionBuilder(
                        new Pose2d(new Vector2d(36, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .build();
        Action goToLaunch2 = drive.actionBuilder(
                        new Pose2d(new Vector2d(15, 45), Math.PI / 2)
                )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .build();
        Action goToLaunch3 = drive.actionBuilder(
                        new Pose2d(new Vector2d(-10, 45), Math.PI / 2)
                )
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-10, 10, 3*Math.PI/4), new Rotation2d(-Math.PI/4, -Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-10, 10), 3*Math.PI/4)
                .build();

        Action fullAuto = new SequentialAction(
                getFirstRow,
                goToLaunch1,
                wait,
                getSecondRow,
                goToLaunch2,
                wait,
                getThirdRow,
                goToLaunch3



        );
        Actions.runBlocking(fullAuto);



//        Trajectory traj = drive.TrajectoryBuilder( 22, Math.toRadians(180)
//                .splineTo(new Vector2d(14, 22), Math.toRadians(160))
//                .waitSeconds(1)
//                .splineTo(new Vector2d(8, 52), Math.toRadians(180))
//                .waitSeconds(1)
//                .build());
    }
}
