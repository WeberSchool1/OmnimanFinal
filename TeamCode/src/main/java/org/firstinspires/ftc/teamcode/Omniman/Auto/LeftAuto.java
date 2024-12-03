package org.firstinspires.ftc.teamcode.Omniman.Auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Omniman.Omniman;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous(name="LeftAuto", group="LeftAuto")
public class LeftAuto extends LinearOpMode {
    private double xPos;
    private double yPos;
    private int  armPositionup=1000;
    private int  armPositionDown=0;
    private int linearPositionup=0;
    private int  linearPositionDown=0;
    Pose2d p;


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
                //Auto Movement code
                //Score first Sample
                man.ArmTargetPos(1000);
                Action MoveHB1st = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(25, -9), Math.toRadians(55))
                        .build();
                Actions.runBlocking(MoveHB1st);
                man.LinearTargetPos(0);
                man.delay(2);
                man.intakePower(-1);
                //Arm Code Start
                xPos=p.position.x;
                yPos=p.position.y;

            }

        }
    }


}
