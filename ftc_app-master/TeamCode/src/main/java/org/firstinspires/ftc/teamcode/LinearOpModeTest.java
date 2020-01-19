package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearOpModeTest extends LinearOpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;


    private void  moveToPoint (double whereIWantToGo){

        double distance = whereIWantToGo - RFMotor.getCurrentPosition();
        telemetry.addData("distance", distance);
        setTankPower(distance/1000);

    }


    private void setTankPower (double power) {
        telemetry.addData("power", power);
        RFMotor.setPower(power);
        LFMotor.setPower(power);
        RBMotor.setPower(power);
        LBMotor.setPower(power);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //Declaring the HardwareMap
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LFMotor");

        //Using RUN_USING_ENCODERS because it changes the speed of the motors depending on the ticks
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       telemetry.addData("distance", RFMotor.getCurrentPosition());
       telemetry.addData("distance", LFMotor.getCurrentPosition());
       telemetry.addData("distance", RBMotor.getCurrentPosition());
       telemetry.addData("distance", LBMotor.getCurrentPosition());

       if(RFMotor.getCurrentPosition() > 0) {

           RFMotor.setTargetPosition(0);
           LFMotor.setTargetPosition(0);
           RBMotor.setTargetPosition(0);
           LBMotor.setTargetPosition(0);

       } else {
           RFMotor.setPower(0);
           LFMotor.setPower(0);
           RBMotor.setPower(0);
           LBMotor.setPower(0);
       }

       waitForStart();

       moveToPoint(813);

    }



}
