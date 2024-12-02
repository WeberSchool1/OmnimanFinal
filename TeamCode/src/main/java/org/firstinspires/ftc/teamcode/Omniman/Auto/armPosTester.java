package org.firstinspires.ftc.teamcode.Omniman.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ArmPositionTester", group="ArmPositionTester")
public class armPosTester extends LinearOpMode {
    private double Armposition;
    private double LinearPosition;
public void Armposititon(double Arm)
{
    Armposition=Arm;
}
public void LinearPosition(double Linear)
{
    LinearPosition=Linear;
}

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("ArmPositon", Armposition);
        telemetry.addData("LinearPositon", LinearPosition);
    }
}
