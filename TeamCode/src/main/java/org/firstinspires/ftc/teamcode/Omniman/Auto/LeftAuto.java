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

                Action MoveHB1st = drive.actionBuilder(new Pose2d(-23.95, -63.55, Math.toRadians(180.00)))
                        .strafeToLinearHeading(new Vector2d(-54.0, -54), Math.toRadians(225))
                        .build();
                Actions.runBlocking(MoveHB1st);
                man.delay(5);
                Action Move1stSample=drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(225)))
                        .splineToConstantHeading(new Vector2d(-34,-34),Math.toRadians(225))
                        .splineToConstantHeading(new Vector2d(-40,-20),Math.toRadians(225))
                        .build();
                Actions.runBlocking(Move1stSample);
                man.delay(3);
                Action MoveHB2ND = drive.actionBuilder(new Pose2d(-40, -20,Math.toRadians(225)))
                        .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(225))
                        .build();
                Actions.runBlocking(MoveHB2ND);
                man.delay(2);




            }

        }
    }


}
