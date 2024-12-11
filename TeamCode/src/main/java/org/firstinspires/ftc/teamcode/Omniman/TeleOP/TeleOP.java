package org.firstinspires.ftc.teamcode.Omniman.TeleOP;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private int armPositionVariable= 0;
    HardwareMap hwMap;
    Pose2d p;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive and Omniman objects
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Man = new Omniman(hardwareMap);
        }

        waitForStart();
        Man.setArmPosition(0);
       //Drive Motors
        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.right_stick_y,
                            -gamepad1.right_stick_x
                    ),

                    -gamepad1.left_stick_x
            ));
            //Noah's controls

            //Intake logic
            if (gamepad1.a) {
                Man.intakePower(1);
            } else if (gamepad1.y) {
                Man.intakePower(0);
            } else if (gamepad1.b) {
                Man.intakePower(.5);
            }
            Man.setLinearPosSlide((int) (200+gamepad1.right_trigger * 2000));
            //LinearSlide Logic
            if(gamepad1.dpad_up)
            {
                armPositionVariable=1750;
            } else if (gamepad1.dpad_down) {
                armPositionVariable=300;
            }else if (gamepad1.dpad_left)
            {
                armPositionVariable=700;
            }else if (gamepad1.dpad_right)
            {
                armPositionVariable=500;
            }else if(gamepad1.start)
            {
                armPositionVariable=-100;
            }
            Man.setArmPosition(armPositionVariable);

        }
        //Drew's controls
            //Ascent arm logic
            if(gamepad2.dpad_up){
                Man.ascentPower(1);
            } else if (gamepad2.dpad_down) {
                Man.ascentPower(-1);
            }else {Man.ascentPower(0);}
            //Snapper code

        // Update telemetry for debugging (optional)
            telemetry.addData("Samp",Man.getIntakeSensor());
            telemetry.update();
        }
    }



