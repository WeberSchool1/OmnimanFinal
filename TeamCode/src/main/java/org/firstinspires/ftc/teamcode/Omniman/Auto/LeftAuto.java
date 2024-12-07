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
    Omniman man;
    private double xPos;
    private double yPos;
    private int armTargetPos;
    private int linearTargetPos;
    Pose2d p;


    MecanumDrive drive;

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
                man.setArmPosition(1000);
                man.delay(.1);
                Action MoveHB1st = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(25, -9), Math.toRadians(55))
                        .build();
                Actions.runBlocking(MoveHB1st);
                man.setLinearSlide(1000);
                man.delay(1.5);
                man.intakePower(0);
                man.delay(1);
                man.intakePower(.5);
                man.setLinearSlide(0);
                man.delay(.5);
                man.setArmPosition(0);
                Action Move1stSample = drive.actionBuilder(new Pose2d(25, -9, Math.toRadians(55)))
                        .strafeToLinearHeading(new Vector2d(10, -9), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(10, -33), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(12, -33), Math.toRadians(0))
                        .build();
                Actions.runBlocking(Move1stSample);
                man.intakePower(1);
                man.delay(1);
                man.intakePower(.5);
                Action MoveHB2nd = drive.actionBuilder(new Pose2d(12, -33, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(25, -9), Math.toRadians(55))
                        .build();
                Actions.runBlocking(MoveHB2nd);
                man.setLinearSlide(1000);
                man.delay(1.5);
                man.intakePower(0);
                man.delay(1);
                man.intakePower(.5);
                man.setLinearSlide(0);
                man.delay(.5);
                man.setArmPosition(0);
                //Arm Code Start
                xPos=p.position.x;
                yPos=p.position.y;

            }

        }
    }



}