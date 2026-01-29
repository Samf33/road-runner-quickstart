package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Maccabot extends LinearOpMode {
    private DcMotor smallLauncherWheels, intake;
    private DcMotorEx mainLauncher2, mainLauncher;
    private CRServo servoLaunchRight, servoLaunchLeft;
    private Servo aim;
    private MecanumDrive drive;
    private Limelight3A limelight;
    private CRServo rgbLight;

    private double launchSpeed = 1.0;
    private boolean isLaunching = false;
    private boolean isShooting = false;
    private boolean isIntaking = false;

    private final double INTAKE_POWER = -0.75;
    private final double SMALL_WHEELS_POWER = 0.9;
    private final double SERVO_LAUNCH_IDLE_POWER = -0.3;
    private final double MINIMUM_SHOOT_RPM = 4000;

    @Override
    public void runOpMode() {
        initDriveAndMotors();
        initLimelight();
        initLight();
        setShoot(false);
    }

    private void initDriveAndMotors() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");

        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");

        aim = hardwareMap.servo.get("aim");

        servoLaunchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mainLauncher2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(100);
    }

    private void initLight() {
        rgbLight = hardwareMap.get(CRServo.class, "rgb");
    }

    public void setIntake(boolean on) {
        isIntaking = on;
        if (on) {
            intake.setPower(INTAKE_POWER);
            smallLauncherWheels.setPower(SMALL_WHEELS_POWER);
        } else {
            intake.setPower(0);
            if (!getIsShooting()) {
                smallLauncherWheels.setPower(0);
            }
        }
    }

    public boolean getIsIntaking() {
        return  isIntaking;
    }

    public void setLaunchSpeed(double speed) {
        launchSpeed = speed;
        if (isLaunching) {
            setLaunch(true);
        }
    }

    public void setLaunch(boolean on) {
        isLaunching = on;
        if (on) {
            mainLauncher.setPower(launchSpeed);
            mainLauncher2.setPower(launchSpeed);
        } else {
            mainLauncher.setPower(0);
            mainLauncher2.setPower(0);
        }
    }

    public boolean getIsLaunching() {
        return isLaunching;
    }

    public void setShoot(boolean on) {
        isShooting = on;
        if (on) {
            servoLaunchRight.setPower(1);
            servoLaunchLeft.setPower(1);
            smallLauncherWheels.setPower(SMALL_WHEELS_POWER);

        } else {
            servoLaunchRight.setPower(SERVO_LAUNCH_IDLE_POWER);
            servoLaunchLeft.setPower(SERVO_LAUNCH_IDLE_POWER);
            if (!getIsIntaking()) {
                smallLauncherWheels.setPower(0);
            }
        }
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public void setDrive(double xVel, double yVel, double rVel) {
        drive.setDrivePowers(
            new PoseVelocity2d(
                new Vector2d(xVel, yVel),
                rVel
            )
        );
    }

    public void stopDrive() {
        setDrive(0,0,0);
    }

    public void setAim(double angle) {
        aim.setPosition(angle);
    }

    public boolean shouldShoot() {
        return mainLauncher.getVelocity() > MINIMUM_SHOOT_RPM
                && mainLauncher2.getVelocity() > MINIMUM_SHOOT_RPM;
    }

    public void setLight(double color) {
        rgbLight.setPower(color);
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }
}
