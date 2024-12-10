package org.firstinspires.ftc.teamcode.Omniman.Auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Omniman.Omniman;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous(name="LeftAuto", group="LeftAuto")
public class LeftAuto extends LinearOpMode {
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
            Omniman man = new Omniman(hardwareMap);
            while (opModeIsActive()) {
                //Auto Movement code
                //Score first Sample
                man.setArmPosition(1650);
                man.setLinearPosSlide(0);
                Action MoveHB1st = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(23, -8), Math.toRadians(55))
                        .build();
                Actions.runBlocking(MoveHB1st);
                man.setLinearPosSlide(2200);
                man.delay(.3);
                man.intakePower(0);
                man.delay(1.5);
                man.intakePower(.5);
                man.setLinearPosSlide(200);
                man.delay(1);
                man.setArmPosition(270);
                Action Move1stSample = drive.actionBuilder(new Pose2d(25, -9, Math.toRadians(55)))
                        .strafeToLinearHeading(new Vector2d(10, -25), Math.toRadians(-10))
                        .strafeToLinearHeading(new Vector2d(10, -31), Math.toRadians(-10))
                        .build();
                Actions.runBlocking(Move1stSample);
                man.setLinearPosSlide(500);
                man.intakePower(1);
                man.delay(1.2);
                man.intakePower(.5);
                man.setArmPosition(1700);
                Action MoveHB2nd = drive.actionBuilder(new Pose2d(12, -33, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(24, -9), Math.toRadians(43))
                        .build();
                Actions.runBlocking(MoveHB2nd);
                man.delay(.5);
                man.setLinearPosSlide(2200);
                man.delay(1);
                man.intakePower(0);
                man.delay(1);
                man.intakePower(.5);
                man.setLinearPosSlide(200);
                man.delay(1);
                man.setArmPosition(280);
                Action Move3rdSample = drive.actionBuilder(new Pose2d(24, -9, Math.toRadians(43)))
                        .strafeToLinearHeading(new Vector2d(16, -25), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(18.5, -32), Math.toRadians(-10))
                        .build();
                Actions.runBlocking(Move3rdSample);
                man.setLinearPosSlide(500);
                man.intakePower(1);
                man.delay(1.2);
                man.intakePower(.5);
                man.setArmPosition(1700);
                Action MoveHB3rd = drive.actionBuilder(new Pose2d(18,-33,Math.toRadians(-10)))
                        .strafeToLinearHeading(new Vector2d(25, -11), Math.toRadians(43))
                        .build();
                Actions.runBlocking(MoveHB3rd);
                man.delay(.5);
                man.setLinearPosSlide(2200);
                man.delay(1);
                man.intakePower(0);
                man.delay(1);
                man.intakePower(.5);
                man.setLinearPosSlide(200);
                man.delay(1);
                man.setArmPosition(300);
                Action Move4thSample = drive.actionBuilder(new Pose2d(24, -10, Math.toRadians(43)))
                        .strafeToLinearHeading(new Vector2d(24, -25), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(26, -35), Math.toRadians(-25))
                        .build();
                Actions.runBlocking(Move4thSample);
                man.setLinearPosSlide(550);
                man.intakePower(1);
                man.delay(1.2);
                man.intakePower(.5);
                man.setArmPosition(1750);
                Action MoveHB4 = drive.actionBuilder(new Pose2d(26, -33.5, Math.toRadians(45)))
                        .strafeToLinearHeading(new Vector2d(25.5, -11), Math.toRadians(60))
                        .build();
                Actions.runBlocking(MoveHB4);
                man.delay(.5);
                man.setLinearPosSlide(2200);
                man.delay(1);
                man.intakePower(0);
                man.delay(1);
                man.intakePower(.5);
                man.setLinearPosSlide(200);
                man.delay(1);
                man.setArmPosition(1000);
                man.delay(1);

            }

        }
    }



}