package org.firstinspires.ftc.teamcode;

public class SecondQualTeleopTarget extends  SecondQualTeleop {
    @Override
    public void runOpMode() throws InterruptedException  {
        runMode = SecondQualTeleopMode.TARGET;
        super.runOpMode();
    }
}