package org.firstinspires.ftc.teamcode;

public class SecondQualTeleopBase extends  SecondQualTeleop {
    @Override
    public void runOpMode() throws InterruptedException  {
        runMode = SecondQualTeleopMode.BASE;
        super.runOpMode();
    }
}