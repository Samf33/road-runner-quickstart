import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Maccabot extends LinearOpMode {
    DcMotor smallLauncherWheels, intake;
    DcMotorEx mainLauncher2, mainLauncher;
    CRServo servoLaunchRight, servoLaunchLeft;


    private double launchSpeed = 1.0;
    private boolean isLaunching = false;

    @Override
    public void runOpMode() {
        initDriveAndMotors();
        waitForStart();
    }

    public void initDriveAndMotors() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        smallLauncherWheels = hardwareMap.dcMotor.get("slWheels");
        mainLauncher = hardwareMap.get(DcMotorEx.class, "ml");
        mainLauncher2 = hardwareMap.get(DcMotorEx.class, "ml2");
        intake = hardwareMap.dcMotor.get("intake");

        servoLaunchLeft = hardwareMap.get(CRServo.class, "slLeft");
        servoLaunchRight = hardwareMap.get(CRServo.class, "slRight");
    }

    public void setIntake(boolean on) {
        if (on) {
            intake.setPower(-0.75);
            smallLauncherWheels.setPower(0.9);
        } else {
            intake.setPower(0);
            smallLauncherWheels.setPower(0);
        }
    }

    public void setLaunchSpeed(double speed) {
        launchSpeed = speed;
        if (isLaunching) {
            setShoot(true);
        }
    }

    public void setShoot(boolean on) {
        isLaunching = on;
        if (on) {
            servoLaunchLeft.setPower(launchSpeed);
            servoLaunchRight.setPower(launchSpeed);
        } else {
            servoLaunchLeft.setPower(0);
            servoLaunchRight.setPower(0);
        }
    }
}
