package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "drive")
public class LaunchingTest extends LinearOpMode {
    DcMotorEx mainLauncher2, mainLauncher;

    boolean isInTest = false;

    @Override
    public void runOpMode() throws InterruptedException {
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");

        waitForStart();

        while (!isStopRequested()) {
            if (!isInTest) {
                if (gamepad1.a) {
                    startTest(1);
                } else if (gamepad1.b) {
                    startTest(0.9);
                } else if (gamepad1.x) {
                    startTest(0.8);
                } else if (gamepad1.y) {
                    startTest(0.7);
                }
            } else {
                if (gamepad1.dpad_down) {
                    endTest();
                }
            }

            telemetry.addData("Motor One RPM", mainLauncher.getVelocity()/ 60);
            telemetry.addData("Motor Two RPM", mainLauncher2.getVelocity() / 60);
        }
    }

    public void startTest(double power) {
        mainLauncher.setPower(power);
        mainLauncher2.setPower(power);
        isInTest = true;
    }

    public void endTest() {
        mainLauncher.setPower(0);
        mainLauncher2.setPower(0);
        isInTest = false;
    }
}
