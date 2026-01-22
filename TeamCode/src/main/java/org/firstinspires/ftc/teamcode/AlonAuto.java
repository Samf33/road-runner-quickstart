package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AlonAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 22, Math.toRadians(180)))
                .splineTo(new Vector2d(14, 22), Math.toRadians(160))
                .waitSeconds(1)
                .splineTo(new Vector2d(8, 52), Math.toRadians(180))
                .waitSeconds(1)
                .build());
    }
}
