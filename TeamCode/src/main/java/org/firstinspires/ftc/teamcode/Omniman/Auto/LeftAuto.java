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
    private double armPositionup=10;
    private double armPositionDown=0;
    private double linearPositionup=10;
    private double linearPositionDown=0;
    Pose2d p;


    MecanumDrive drive;
    Omniman man;
    //AutoArmControl Arms;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-23.95, -63.55, 180));
            man = new Omniman(hardwareMap);
            while (opModeIsActive()) {
                //Auto Movement code
                //Score first Sample
                Action MoveHB1st = drive.actionBuilder(new Pose2d(-23.95, -63.55, Math.toRadians(180.00)))
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(135))
                        .build();
                Actions.runBlocking(MoveHB1st);
              /*  man.delay(3);
                //Grab Second Sample
                Action Move1stSample=drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-34,-34),Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(-40,-20),Math.toRadians(135))
                        .build();
                Actions.runBlocking(Move1stSample);
                man.delay(2);
                //Score Second Sample
                Action MoveHB2ND = drive.actionBuilder(new Pose2d(-40, -20,Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(135))
                        .build();
                Actions.runBlocking(MoveHB2ND);
                man.delay(3);
                //Grab 3rd Sample
                Action Move2NDSample=drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-54, -20), Math.toRadians(135))
                        .build();
                Actions.runBlocking(Move2NDSample);
                man.delay(2);
                //Score 3rd Sample
                Action MoveHB3RD=drive.actionBuilder(new Pose2d(-54, -20, Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(135))
                        .build();
                Actions.runBlocking(MoveHB3RD);
                man.delay(3);
                //Grab 4th Sample
                Action Move4thSample=drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-65, -20),Math.toRadians(135))
                        .build();
                Actions.runBlocking(Move4thSample);
                man.delay(2);
                //Score 4th Sample
                Action MoveHB4th=drive.actionBuilder(new Pose2d(-65, -20, Math.toRadians(135)))
                        .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(135))
                        .build();
                Actions.runBlocking(MoveHB4th);
                man.delay(3);
                //Park
                Action Park=drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(135)))
                        .strafeToLinearHeading(new Vector2d(-25, -11), Math.toRadians(360))
                        .build();
                Actions.runBlocking(Park);
                //End Movement code*/
                //Arm Code Start
                xPos=p.position.x;
                yPos=p.position.y;
                 if((xPos<-50)&&(yPos<-50))
                 {
                    man.ArmTargetPos(armPositionup);
                    man.delay(.5);
                    man.LinearTargetPos(linearPositionup);
                    man.delay(1);
                    man.intakePower(-1);

                 }else if ((yPos>-25)){
                     man.intakePower(1);
                     man.ArmTargetPos(armPositionDown);
                     man.LinearTargetPos(linearPositionDown);
                 }else
                 {
                     man.intakePower(0);
                     man.ArmTargetPos(armPositionDown);
                     man.LinearTargetPos(linearPositionDown);
                 }

            }

        }
    }


}
