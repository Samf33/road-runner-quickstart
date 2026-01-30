package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(group = "drive")

public class TestingSetRPM extends LinearOpMode {
    DcMotorEx motor;
    double velocity;
    @Override
    public void runOpMode() throws InterruptedException {
        double ticksPerRev = 383.6;
        double targetRPM = 300;



        motor = hardwareMap.get(DcMotorEx.class, "test");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 12));
        while(!isStopRequested()) {
            velocity = (targetRPM * ticksPerRev) / 60.0;
            motor.setVelocity(velocity);
            if(gamepad1.dpad_up) {
                targetRPM += 5;
            }
            if(gamepad1.dpad_down) {
                targetRPM -= 5;
            }
            double vel = motor.getVelocity(); // ticks/sec
            double rpm = (vel * 60) / 383.6;
            telemetry.addData("RPM", rpm);
            telemetry.addData("targed", targetRPM);
            telemetry.update();
        }
    }

}
