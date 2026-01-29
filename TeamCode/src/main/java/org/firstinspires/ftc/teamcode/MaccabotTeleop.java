package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "drive")
public class MaccabotTeleop extends Maccabot {
    @Override
    public void runOpMode() {
        super.runOpMode();
        setLaunch(true);
        while (!isStopRequested()) {
            setDrive(-gamepad1.left_stick_y, - gamepad1.left_stick_x, -gamepad1.right_stick_x); // DRIVE

            if (gamepad1.a) { // LAUNCH
                setLaunch(!getIsLaunching());
            }

            if (gamepad1.left_trigger >= 0.3) { // SHOOT
                if (!getIsShooting()) {
                    setShoot(true);
                }
            } else {
                if (getIsShooting()) {
                    setShoot(false);
                }
            }

            if (gamepad1.right_trigger >= 0.3) { // INTAKE
                if (!getIsIntaking()) {
                    setIntake(true);
                }
            } else {
                if (getIsIntaking()) {
                    setIntake(false);
                }
            }

            if (shouldShoot()) {
                setLight(0.5);
            } else {
                setLight(0.277);
            }

            telemetry.addData("IsLaunching", getIsLaunching());
            telemetry.addData("IsShooting", getIsShooting());
            telemetry.addData("IsIntaking", getIsIntaking());


            updatePoseEstimate();
        }
    }
}
