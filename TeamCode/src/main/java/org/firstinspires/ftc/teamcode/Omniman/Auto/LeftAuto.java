package org.firstinspires.ftc.teamcode.Omniman.Auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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

                Action trajectory0 = drive.actionBuilder(new Pose2d(-9.86, -66.66, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(-52.72, -66.81), Math.toRadians(180.20))
                        .splineTo(new Vector2d(-41.45, -26.77), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-53.31, -54.35), Math.toRadians(230.00))
                        .build();
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
