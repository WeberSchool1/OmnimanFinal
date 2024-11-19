package org.firstinspires.ftc.teamcode.Omniman;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Omniman.Auto.AutoArmControl;
import org.firstinspires.ftc.teamcode.Omniman.TeleOP.TeleOP;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

public class Omniman {

    // Motor Variables
    private int armCurrentPosition=0;
    private int linearCurrentPosition=0;
    private int specimenArmPosition=0;
    private DcMotor linearSlide;
    private DcMotor armPosition;
    private DcMotor specimenArm;

    // Servo Variables
    private Servo intake;
    private Servo specimenAdjuster;
    private Servo yPodraiser;
    private Servo xPodraiser;

    // Control Objects
    private MecanumDrive drive;
    private TeleOP driverArms = new TeleOP(); // Default initialization
    private AutoArmControl autoArms;

    // Constructor
    public Omniman(HardwareMap hwMap) {
        this(hwMap, new Pose2d(0, 0, 0));
    }

    public Omniman(HardwareMap hwMap, Pose2d pose) {
        // Ensure mandatory objects are created during initialization
        drive = new MecanumDrive(hwMap, pose);
        autoArms = new AutoArmControl(hwMap);

        // Initialize motors
        armPosition = hwMap.dcMotor.get("arm_position");
        armPosition.setDirection(DcMotorSimple.Direction.REVERSE);
        armPosition.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPosition.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armCurrentPosition=armPosition.getCurrentPosition();

        linearSlide = hwMap.dcMotor.get("linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearCurrentPosition=linearSlide.getCurrentPosition();

        specimenArm = hwMap.dcMotor.get("specimen_arm");
        specimenArm.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        specimenArmPosition=specimenArm.getCurrentPosition();

        // Initialize servos
        intake = hwMap.servo.get("intake");


        // Set initial servo positions
        intake.setPosition(0.5);
    }

    public void updateMotorsAndServos() {
        // Update motor powers
        armPosition.setPower(autoArms.getArmPower() + driverArms.getArmPower());
        linearSlide.setPower(autoArms.getLinearSlidePower() + driverArms.getLinearPower());
        specimenArm.setPower(autoArms.getSpecimenArmPower() + driverArms.getSpecimenPower());

        // Update servo positions
        intake.setPosition(driverArms.getIntakePower());
    }

    public void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {
            // Busy wait
        }
    }

    // Getter methods for motor positions
    public int getArmPosition() {
        return armCurrentPosition;
    }

    public int getLinearArmPosition() {
        return linearCurrentPosition;
    }

    public int getSpecimenArmPosition() {
        return specimenArmPosition;
    }
}
