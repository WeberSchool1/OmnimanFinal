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


    // Servo Variables
    private Servo intake;
    private Servo specimenAdjuster;
    private Servo yPodraiser;
    private Servo xPodraiser;
    //Sensors
    private ColorSensor intakeSensor;
    //Funny colors
    private int red= intakeSensor.red();
    private int green= intakeSensor.green();
    private int blue= intakeSensor.blue();
    private float[] hsvValues = new float[3];

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
        armPosition.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPosition.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPosition.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        linearSlide = hwMap.dcMotor.get("linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setDirection((DcMotorSimple.Direction.REVERSE));

        specimenArm = hwMap.dcMotor.get("specimen_arm");
        specimenArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        specimenArm.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos
        intake = hwMap.servo.get("intake");
        specimenAdjuster=hwMap.servo.get("specimen_adjuster");


        // Set initial servo positions
        intake.setPosition(0.5);
        specimenAdjuster.setPosition((0.5));

        //Initialize sensors
        intakeSensor = hwMap.colorSensor.get("intakeSensor");
        int red= intakeSensor.red(); int green= intakeSensor.green(); int blue= intakeSensor.blue();

        float[] hsvValues = new float[3];
        Color.RGBToHSV(red, green, blue, hsvValues);
    }



public void ArmTargetPos(double TargetPos)
{
    if(((armPosition.getCurrentPosition()-TargetPos)<0)&&((armPosition.getCurrentPosition()-TargetPos)<100)){
        armPosition.setPower(1);
    } else if ((armPosition.getCurrentPosition()-TargetPos)>0){
       armPosition.setPower(-1);
    }else{
      armPosition.setPower(0);
    }
}
public void LinearTargetPos(double TargetPos){
    if((linearSlide.getCurrentPosition()-TargetPos)<0){
        linearSlide.setPower(1);
    }
    else if (linearSlide.getCurrentPosition()-TargetPos>0){
        linearSlide.setPower(-1);
    }else{
        if (armPosition.getCurrentPosition()<2500)
        {
            linearSlide.setPower(-.1);
        } else if (armPosition.getCurrentPosition()>3500) {
           linearSlide.setPower(.1);

        }
        else {
            if(armPosition.getCurrentPosition()<5000)
            {            if(linearSlide.getCurrentPosition()>2500){linearSlide.setPower(-.4);}}
            linearSlide.setPower(0);
        }
    }
}
public void SpecPos(double TargetPos)
{
     if((specimenArm.getCurrentPosition()-TargetPos)<0)
     {
         specimenArm.setPower(1);
     } else if ((specimenArm.getCurrentPosition()-TargetPos)>0) {
         specimenArm.setPower(-1);
     }else{
         specimenArm.setPower(0);
     }


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

        // Update servo positions
    public void intakePower(double Power) {
        intake.setPosition(Power);
    }
    public void specimenAdjusterPower(double Power){
        specimenAdjuster.setPosition(Power);
    }

    public void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {
            // Busy wait
        }
    }


}
