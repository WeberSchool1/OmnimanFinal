package org.firstinspires.ftc.teamcode.Omniman.TeleOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    HardwareMap hwMap;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive and Omniman objects
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Man = new Omniman(hardwareMap);
        }

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.right_stick_x,
                                gamepad1.right_stick_y
                        ),
                        gamepad1.left_stick_x
                ));
            // Initialize motors
            if (gamepad1.right_trigger > 0) {
                Man.armPositionPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                Man.armPositionPower(-gamepad1.left_trigger);
            } else {
                Man.armPositionPower(.01);
            }

            // Linear slide logic
            if (gamepad1.right_bumper) {
                Man.linearPower(1);
            } else if (gamepad1.left_bumper) {
                Man.linearPower(-1);
            } else {
               Man.linearPower(0);
            }

            // Specimen arm logic
            if (gamepad1.dpad_up) {
                Man.specimenPower(1);
            } else if (gamepad1.dpad_down) {
                Man.specimenPower(-1);;
            } else {
                Man.specimenAdjusterPower(.5);
                Man.specimenPower(0);            }

            // Intake logic
            if (gamepad1.a) {
                Man.intakePower(1);
            } else if (gamepad1.y) {
                Man.intakePower(0);
            } else if (gamepad1.b) {
                Man.intakePower(.5);
            }



            // Arm position logic


            // Update telemetry for debugging (optional)
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Linear Power", linearPower);
            telemetry.addData("Specimen Power", specimenPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Specimen Adjuster", specimenadjuster);
            telemetry.update();
        }
    }


}
