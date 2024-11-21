package org.firstinspires.ftc.teamcode.Omniman.Auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Omniman.Omniman;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous(name="LeftAuto", group="LeftAuto")
public class LeftAuto extends LinearOpMode {
    private double ARMTARGETPOS;
    private double LINEARTARGETPOS;
    MecanumDrive drive;
    Omniman man;
    //AutoArmControl Arms;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            man = new Omniman(hardwareMap);
            while (opModeIsActive()) {

                Action MoveHB1st = drive.actionBuilder(new Pose2d(-24.10, -63.55, Math.toRadians(180.00)))
                        .splineTo(new Vector2d(-52.87, -54.20), Math.toRadians(225.00))
                        .build();
                Actions.runBlocking(MoveHB1st);
                man.delay(5);



            }

        }
    }
    public double getARMTARGETPOS(){
      return ARMTARGETPOS;
    }
    public double getLINEARTARGETPOS(){
        return LINEARTARGETPOS;
    }
}
