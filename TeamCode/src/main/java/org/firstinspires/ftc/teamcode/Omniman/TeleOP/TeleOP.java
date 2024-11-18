package org.firstinspires.ftc.teamcode.Omniman.TeleOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Omniman.Omniman;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@TeleOp(name="TeleOP", group="TeleOP")
public class TeleOP extends LinearOpMode{

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    MecanumDrive Drive;
    Omniman Man;
    private double armPower;
    private double linearPower;
    private double specimenPower;
    private double intakePower;
    private double specimenadjuster;
    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Man = new Omniman(hardwareMap);

            while (opModeIsActive()){
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

            }
            //armPos code
            if(gamepad1.right_trigger>0)
            {
                armPower=(gamepad1.right_trigger);

            } else if (gamepad1.left_trigger>0) {
                armPower=(-(gamepad1.left_trigger));
            }else{
                armPower=0;
            }
            //linearSlide code
            if(gamepad1.right_bumper)
            {
                linearPower=1;
            } else if (gamepad1.left_bumper) {
                linearPower=-1;
            }else
            {
                linearPower=0;
            }
            //SpecimenArm code
            if(gamepad1.dpad_up)
            {
                specimenPower=1;
            } else if (gamepad1.dpad_down)
            {
                specimenPower=0;
            }else if(gamepad1.dpad_left)
            {
                specimenadjuster=1;
            } else if (gamepad1.dpad_right)
            {
                specimenadjuster=0;
            }else
            {
                specimenadjuster=.5;
                specimenPower=.5;
            }


        }

    }

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
