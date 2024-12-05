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
                //LinearSlide Logic
                if (gamepad1.right_bumper) {
                    Man.linearPower(1);
                } else if (gamepad1.left_bumper) {
                    Man.linearPower(-1);
                } else {
                    Man.linearPower(0);
                }

                if (gamepad1.right_trigger > 0) {
                    Man.armPositionPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    Man.armPositionPower(-gamepad1.left_trigger);
                } else {
                    Man.armPositionPower(.01);
                }

            if (gamepad1.dpad_up) {
                Man.specimenPower(1);
            } else if (gamepad1.dpad_down) {
                Man.specimenPower(-1);;
            } else {
                Man.specimenPower(0);
            }
        }
        //Drew's controls
            //Ascent arm logic
            if(gamepad2.dpad_up){
                Man.ascentPower(1);
            } else if (gamepad2.dpad_down) {
                Man.ascentPower(-1);
            }else {Man.ascentPower(0);}
            //Snapper code
        Action Snap0= drive.actionBuilder(new Pose2d(p.position.x, p.position.y, 0))
                .build();
            if (gamepad2.left_stick_button){
                Actions.runBlocking(Snap0);
            }
            Action Snap180=drive.actionBuilder(new Pose2d(p.position.x,p.position.y, 180))
                    .build();
            if(gamepad2.right_stick_button)
            {
                Actions.runBlocking(Snap180);
            }


        // Update telemetry for debugging (optional)
            telemetry.addData("Samp",Man.getIntakeSensor());
            telemetry.update();
        }
    }



