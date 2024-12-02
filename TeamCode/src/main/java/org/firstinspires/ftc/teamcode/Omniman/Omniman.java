package org.firstinspires.ftc.teamcode.Omniman;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode.Omniman.TeleOP.TeleOP;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

import java.lang.annotation.Target;

public class Omniman {

    // Motor Variables
    private DcMotor linearSlide;
    private DcMotor armPosition;
    private DcMotor specimenArm;
    private DcMotor ascentArm;


    // Servo Variables
    private Servo intake;
    private Servo specimenAdjuster;
    private Servo yPodraiser;
    private Servo xPodraiser;
    //Sensors
    private ColorSensor intakeSensor;
    //Funny colors


    // Control Objects
    private MecanumDrive drive;
    private TeleOP driverArms = new TeleOP(); // Default initialization


    // Constructor
    public Omniman(HardwareMap hwMap) {
        this(hwMap, new Pose2d(0, 0, 0));
    }

    public Omniman(HardwareMap hwMap, Pose2d pose) {
        // Ensure mandatory objects are created during initialization
        drive = new MecanumDrive(hwMap, pose);


        // Initialize motors
        armPosition = hwMap.dcMotor.get("arm_position");
        armPosition.setDirection(DcMotorSimple.Direction.REVERSE);
        armPosition.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPosition.setTargetPosition(0);

        armPosition.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        linearSlide = hwMap.dcMotor.get("linear_slide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setTargetPosition(0);
        linearSlide.setDirection((DcMotorSimple.Direction.REVERSE));

        specimenArm = hwMap.dcMotor.get("specimen_arm");
        specimenArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        specimenArm.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize servos
        intake = hwMap.servo.get("intake");

        if(armPosition.getCurrentPosition()-armPosition.getTargetPosition()<0)
        {
            armPosition.setPower(1);
        } else if ((armPosition.getCurrentPosition()-armPosition.getTargetPosition())>0) {
            armPosition.setPower(-1);

        }else{
            armPosition.setPower(0);
        }
        // Initialize servos
        intake = hwMap.servo.get("intake");
        if((linearSlide.getCurrentPosition()-linearSlide.getTargetPosition())<0)
        {
            linearSlide.setPower(1);
        } else if ((linearSlide.getCurrentPosition()- linearSlide.getTargetPosition()<0)){
            linearSlide.setPower(-1);

        }else{
            linearSlide.setPower(0);
        }


    }

public void ArmTargetPos(int TargetPos){
        armPosition.setTargetPosition(TargetPos);
}

public void LinearTargetPos(int TargetPos){
        armPosition.setTargetPosition(TargetPos);
}


   public void armPositionPower(double Power) {
       // Update motor powers
       armPosition.setPower((Power));

   }
   public void linearPower(double Power) {
       linearSlide.setPower((Power));
   }
   public void specimenPower(double Power) {
       specimenArm.setPower(Power);
   }
   public void ascentPower(double Power){
        ascentArm.setPower(Power);
   }

        // Update servo positions
    public void intakePower(double Power) {
        intake.setPosition(Power);
    }


    public void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {
            // Busy wait
        }
    }


}
