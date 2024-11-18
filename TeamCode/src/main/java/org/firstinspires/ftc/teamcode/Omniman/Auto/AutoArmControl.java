package org.firstinspires.ftc.teamcode.Omniman.Auto;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Omniman.Omniman;

public class AutoArmControl {
    private com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    Omniman omniman;
    //Arm Position imports
    double armPosition =omniman.getArmPosition();
    double targetArmPosition = omniman.getArmTargetPos();
    double armPower;
    //end Arm Position imports

    //Linear Slide imports
    double linearSlidePosition =omniman.getLinearArmPosition();
    double linearSlideTargetPosition =omniman.getLinearTargetPos();
    double linearSlidePower;

    //end Linear Slide imports

    //Speciman Arm imports
    double specimanArmPosition =omniman.getSpecimenArmPosition();
    double specimenArmTargetPosition= omniman.getSpecimenTargetPos();
    double specimenArmPower;

    double SpecimenArmAdjuster = omniman.getSpecimenAdjuster();
    //end Specimen Arm imports


    public AutoArmControl(HardwareMap hwMap)
    {

    //Arm Position movement
    if ((armPosition-targetArmPosition)>0&&(armPosition-targetArmPosition)>10)
    {
        armPower=1;
    } else if ((armPosition-targetArmPosition)<0&&(armPosition-targetArmPosition)<10) {
        armPower=-1;
    }else {
        armPower=.05;

    }

    if((linearSlidePosition-linearSlideTargetPosition)>0 && (linearSlidePosition-linearSlideTargetPosition)>10)
    {
        linearSlidePower=1;
    }else if((linearSlidePosition-linearSlideTargetPosition)<0 && (linearSlidePosition-linearSlideTargetPosition)<10)
    {
        linearSlidePower=-1;
    }else{
        if (armPosition<2500)
        {
            linearSlidePower=-.1;
        } else if (armPosition>2500 && armPosition<3500) {
            linearSlidePower=0;
        }else if (armPosition>3500){
            linearSlidePower=.1;
        }

    }
    if((specimanArmPosition-specimenArmTargetPosition)>0&& (specimanArmPosition-specimenArmTargetPosition)>10)
    {
        specimenArmPower=1;
    } else if ((specimanArmPosition-specimenArmTargetPosition)<0&& (specimanArmPosition-specimenArmTargetPosition)<10)
    {
        specimenArmPower=-1;

    }else
        specimenArmPower=.1;
    }

    public double getArmPower() {
        return armPower;
    }

    public double getLinearSlidePower() {
        return linearSlidePower;
    }

    public double getSpecimenArmPower(){
        return specimenArmPower;
    }
}




