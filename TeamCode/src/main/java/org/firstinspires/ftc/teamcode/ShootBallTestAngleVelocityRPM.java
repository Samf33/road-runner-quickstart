package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class ShootBallTestAngleVelocityRPM extends LinearOpMode {

    final double GEAR_RATIO = 29.0 / 300.0;

    final double SERVO_MIN_DEG = 30.0;
    final double SERVO_MAX_DEG = 60.0;

    boolean downDebounce, upDebounce, leftDebounce, rightDebounce, xDebounce;

    double currentAngle = 0.0;
    double currentPower = 0.0;

    boolean isShooting = false;

    Servo launcherAngleServo;

    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");

        launcherAngleServo = hardwareMap.get(Servo.class, "launcherAngleServo");

        final double LAUNCHER_ANGLE_MIN = SERVO_MIN_DEG * GEAR_RATIO;
        final double LAUNCHER_ANGLE_MAX = SERVO_MAX_DEG * GEAR_RATIO;

        waitForStart();

        launcherAngleServo.setPosition(0.0);

        while (!isStopRequested()) {
            boolean shot = false;

            if (gamepad1.dpad_up) {
                if (!upDebounce) {
                    upDebounce = true;
                    currentAngle += 0.05;
                }
            } else
                upDebounce = false;

            if (gamepad1.dpad_down) {
                if (!downDebounce) {
                    downDebounce = true;
                    currentAngle -= 0.05;
                }
            } else
                downDebounce = false;

            if (gamepad1.dpad_left) {
                if (!leftDebounce) {
                    leftDebounce = true;
                    currentPower = 0.9;
                }
            } else
                leftDebounce = false;

            if (gamepad1.dpad_right) {
                if (!rightDebounce) {
                    rightDebounce = true;
                    currentPower = 0.5;
                }
            } else
                rightDebounce = false;

            double launcherAngle = LAUNCHER_ANGLE_MIN + currentAngle * (LAUNCHER_ANGLE_MAX - LAUNCHER_ANGLE_MIN);

            double servoAngle = launcherAngle / GEAR_RATIO;

            double servoPos = Math.max(0.0, Math.min(1.0, servoAngle / 180.0));

            launcherAngleServo.setPosition(servoPos);

            if (gamepad1.x) {
                if (!xDebounce) {
                    xDebounce = true;
                    isShooting = !isShooting;
                }
            } else
                xDebounce = false;

            if (isShooting) {
                mainLauncher.setPower(currentPower);
                mainLauncher2.setPower(currentPower);
            } else {
                mainLauncher.setPower(0);
                mainLauncher2.setPower(0);
            }

            if (gamepad1.right_trigger >= 0.3) {
                servoLaunchLeft.setPower(1);
                servoLaunchRight.setPower(1);
            } else
            {
                servoLaunchLeft.setPower(-0.3);
                servoLaunchRight.setPower(-0.3);
            }

            telemetry.addData("Launcher Angle (deg)", launcherAngle);
            telemetry.addData("Launcher Power", currentPower);
            telemetry.addData(
                    "motor speed (RPM)",
                    "L1: " + 60 * (mainLauncher.getVelocity() / 28) +
                            " L2: " + 60 * (mainLauncher2.getVelocity() / 28)
            );
            
            if (gamepad1.right_trigger >= 0.3)
                telemetry.update();
        }
    }
}
