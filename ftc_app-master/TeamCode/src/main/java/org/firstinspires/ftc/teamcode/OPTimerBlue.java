package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous
public class OPTimerBlue extends LinearOpMode {

    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private RevColorSensorV3 sensorColor;
    private CRServo servoArm;
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = -0.5;
    static final double FAST_SPEED = -0.9;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    private void intakeSkystone() {
        IntakeL.setPower(-0.5*FORWARD_SPEED);
        IntakeR.setPower(0.5*FORWARD_SPEED);
    }

    private void connectToHardwareMap() {
        //connecting program names to hardware map names on robot controller phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "Color");
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void moveForward(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(FORWARD_SPEED);
            LFMotor.setPower(FORWARD_SPEED);
            RBMotor.setPower(FORWARD_SPEED);
            LBMotor.setPower(FORWARD_SPEED);
        }
    }
    private void moveForwardSlow(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(0.5*FORWARD_SPEED);
            LFMotor.setPower(0.5*FORWARD_SPEED);
            RBMotor.setPower(0.5*FORWARD_SPEED);
            LBMotor.setPower(0.5*FORWARD_SPEED);
        }
    }
    private void moveForwardFast(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(FAST_SPEED);
            LFMotor.setPower(FAST_SPEED);
            RBMotor.setPower(FAST_SPEED);
            LBMotor.setPower(FAST_SPEED);
        }
    }
    private void moveBackward(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(-FORWARD_SPEED);
            LFMotor.setPower(-FORWARD_SPEED);
            RBMotor.setPower(-FORWARD_SPEED);
            LBMotor.setPower(-FORWARD_SPEED);
        }
    }
    private void moveBackwardSlow(){
        RFMotor.setPower(-0.5*FORWARD_SPEED);
        LFMotor.setPower(-0.5*FORWARD_SPEED);
        RBMotor.setPower(-0.5*FORWARD_SPEED);
        LBMotor.setPower(-0.5*FORWARD_SPEED);
    }
    private void moveBackwardFast(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(-FAST_SPEED);
            LFMotor.setPower(-FAST_SPEED);
            RBMotor.setPower(-FAST_SPEED);
            LBMotor.setPower(-FAST_SPEED);
        }
    }
    private void moveUpLeftDiagonal(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(FORWARD_SPEED);
            RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBMotor.setPower(FORWARD_SPEED);
        }
    }
    private void moveBackLeftDiagonal(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            LFMotor.setPower(-FORWARD_SPEED);
            LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBMotor.setPower(-FORWARD_SPEED);
        }
    }
    private void stopRobot(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            LFMotor.setPower(0);
            LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBMotor.setPower(0);
            LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFMotor.setPower(0);
            RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBMotor.setPower(0);
            RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void moveLeft( double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(FORWARD_SPEED);
            LFMotor.setPower(-FORWARD_SPEED);
            RBMotor.setPower(-FORWARD_SPEED);
            LBMotor.setPower(FORWARD_SPEED);
        }
    }

    private void moveLeftFast(double input) {
        while (opModeIsActive() && runtime.seconds() < input){
            RFMotor.setPower(FAST_SPEED);
            LFMotor.setPower(-FAST_SPEED);
            RBMotor.setPower(-FAST_SPEED);
            LBMotor.setPower(FAST_SPEED);
        }
    }

    private void moveLeftSlow(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(0.75*FORWARD_SPEED);
            LFMotor.setPower(-0.75*FORWARD_SPEED);
            RBMotor.setPower(-0.75*FORWARD_SPEED);
            LBMotor.setPower(0.75*FORWARD_SPEED);
        }
    }
    private void moveRight(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(-FORWARD_SPEED);
            LFMotor.setPower(FORWARD_SPEED);
            RBMotor.setPower(FORWARD_SPEED);
            LBMotor.setPower(-FORWARD_SPEED);
        }
    }
    private void moveRightFast(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            RFMotor.setPower(-FAST_SPEED);
            LFMotor.setPower(FAST_SPEED);
            RBMotor.setPower(FAST_SPEED);
            LBMotor.setPower(-FAST_SPEED);
        }
    }
    private void turnRight(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            LFMotor.setPower(FORWARD_SPEED);
            LBMotor.setPower(FORWARD_SPEED);
            RFMotor.setPower(-FORWARD_SPEED);
            RBMotor.setPower(-FORWARD_SPEED);
        }
    }
    private void turnLeft(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            LFMotor.setPower(-FORWARD_SPEED);
            LBMotor.setPower(-FORWARD_SPEED);
            RFMotor.setPower(FORWARD_SPEED);
            RBMotor.setPower(FORWARD_SPEED);
        }
    }
    private void colorSense() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }

    private void moveArmDown(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            servoArm.setPower(FAST_SPEED);
        }
    }

    private void moveArmUp(double input) {
        while (opModeIsActive() && runtime.seconds() < input) {
            servoArm.setPower(-FAST_SPEED);
        }
    }
    private void stopArm() {
        servoArm.setPower(0);
    }

    private void senseBlue(){
        while (opModeIsActive() && (hsvValues[0] < 200.0 || hsvValues[0] > 220.0)){
            colorSense();
            moveBackwardSlow();
        }
    }

    private void senseRed(){
        while (opModeIsActive() && (hsvValues[0] < 15.0 || hsvValues[0] > 35.0)){
            colorSense();
            moveBackwardSlow();
        }
    }
    private void getStone(){

        moveRightFast(0.957);

        /*runtime.reset();
        turnRight(0.01);*/

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveArmDown(0.5);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveLeftFast(0.90);

        runtime.reset();
        stopRobot(0.2);



    }

    @Override

    public void runOpMode() {
        connectToHardwareMap();
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //First Stone

        runtime.reset();
        getStone();

        runtime.reset();
        moveForwardFast(1.2);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveArmUp(0.5);

        runtime.reset();
        turnLeft(0.25);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveLeftFast(0.6);

        runtime.reset();
        stopRobot(0.2);

        //Second Stone

        runtime.reset();
        moveBackwardFast(1.13);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveLeftFast(0.35);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        getStone();

        runtime.reset();
        moveForwardFast(1.3);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        moveArmUp(0.6);

        runtime.reset();
        stopRobot(0.2);

        runtime.reset();
        turnLeft(0.1);

        runtime.reset();
        moveLeft(1.0);

        runtime.reset();
        stopRobot(0.2);

        senseBlue();

    }

}
