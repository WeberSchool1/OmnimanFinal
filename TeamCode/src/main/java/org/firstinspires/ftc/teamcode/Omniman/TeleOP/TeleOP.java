package org.firstinspires.ftc.teamcode.Omniman.TeleOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Omniman.Omniman;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "TeleOP", group = "TeleOP")
public class TeleOP extends LinearOpMode {

    // Class-level variables
    private MecanumDrive drive;
    private Omniman Man;
    private double armPower = 0;
    private double linearPower = 0;
    private double specimenPower = 0;
    private double intakePower = 0;
    private double specimenadjuster = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive and Omniman objects
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Man = new Omniman(hardwareMap);
        }

        waitForStart();

        while (opModeIsActive()) {
            // Drive logic
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // Arm position logic
            if (gamepad1.right_trigger > 0) {
                armPower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0) {
                armPower = -gamepad1.left_trigger;
            } else {
                armPower = -0.1;
            }

            // Linear slide logic
            if (gamepad1.right_bumper) {
                linearPower = 1;
            } else if (gamepad1.left_bumper) {
                linearPower = -1;
            } else {
                linearPower = 0;
            }

            // Specimen arm logic
            if (gamepad1.dpad_up) {
                specimenPower = 1;
            } else if (gamepad1.dpad_down) {
                specimenPower = 0;
            } else if (gamepad1.dpad_left) {
                specimenadjuster = 1;
            } else if (gamepad1.dpad_right) {
                specimenadjuster = 0;
            } else {
                specimenadjuster = 0.5;
                specimenPower = 0.5;
            }

            // Intake logic
            if (gamepad1.a) {
                intakePower = -1;
            } else if (gamepad1.y) {
                intakePower = 0;
            } else if (gamepad1.b) {
                intakePower = 0.5;
            }

            // Update telemetry for debugging (optional)
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Linear Power", linearPower);
            telemetry.addData("Specimen Power", specimenPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Specimen Adjuster", specimenadjuster);
            telemetry.update();
        }
    }

    // Getter methods
    public double getArmPower() {
        return armPower;
    }

    public double getLinearPower() {
        return linearPower;
    }

    public double getSpecimenPower() {
        return specimenPower;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getSpecimenadjuster() {
        return specimenadjuster;
    }
}
