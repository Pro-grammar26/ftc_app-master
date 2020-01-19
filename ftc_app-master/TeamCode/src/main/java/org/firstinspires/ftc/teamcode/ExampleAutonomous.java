package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "ExampleAutonomous", group = "Example")
public class ExampleAutonomous extends LinearOpMode {

    // Name all motors you will be using
    DcMotor RfMotor;
    DcMotor RbMotor;
    DcMotor LfMotor;
    DcMotor LbMotor;
    Servo pickerUpper;

    @Override
    public void runOpMode() {
        hardwareMap.dcMotor.get("RfMotor");
        hardwareMap.dcMotor.get("RbMotor");// this is what will show up on the DS
        hardwareMap.dcMotor.get("LfMotor");
        hardwareMap.dcMotor.get("LbMotor");
        hardwareMap.servo.get("pickerUpper");


        // The left motors will be set to Forward
        LfMotor.setDirection(DcMotor.Direction.FORWARD);
        LbMotor.setDirection(DcMotor.Direction.FORWARD);

        // The right motors will be set to Reverse
        RfMotor.setDirection(DcMotor.Direction.REVERSE);
        RbMotor.setDirection(DcMotor.Direction.REVERSE);

        // starting with the servo lowered so it can fit in the 18 by 18 box
        pickerUpper.setPosition(0);

        waitForStart();
        // this is how fast each motor will be moving
        RfMotor.setPower(1);
        RbMotor.setPower(1);
        LfMotor.setPower(1);
        LbMotor.setPower(1);

        // the robot will run for 2 seconds
        sleep(2000);

        // this is what makes the robot stop
        RfMotor.setPower(0);
        RbMotor.setPower(0);
        LfMotor.setPower(0);
        LbMotor.setPower(0);

        // This will make the robot turn right
        RfMotor.setPower(.5);
        RbMotor.setPower(.5);
        LfMotor.setPower(-.5);
        LbMotor.setPower(-.5);

        sleep(2000);// stop

        // this is what makes the robot stop
        RfMotor.setPower(0);
        RbMotor.setPower(0);
        LfMotor.setPower(0);
        LbMotor.setPower(0);

        pickerUpper.setPosition(1);

        sleep(1000);

        pickerUpper.setPosition(-1);

        Double Runtime = getRuntime();
        if (Runtime < 15) {
            RfMotor.setPower(2);
            RbMotor.setPower(2);
            LfMotor.setPower(2);
            LbMotor.setPower(2);

            sleep(1000);

            RfMotor.setPower(0);
            RbMotor.setPower(0);
            LfMotor.setPower(0);
            LbMotor.setPower(0);

        } else {
            RfMotor.setPower(0);
            RbMotor.setPower(0);
            LfMotor.setPower(0);
            LbMotor.setPower(0);
        }
    }
}




