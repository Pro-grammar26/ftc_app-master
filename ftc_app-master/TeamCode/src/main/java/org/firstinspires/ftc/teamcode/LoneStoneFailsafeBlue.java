package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous
public class LOneStoneFailsafeBlue extends LinearOpMode{

    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private RevColorSensorV3 sensorColor;
    private CRServo servoArm;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = -0.5;
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

    private void moveForward() {
        //All motors move forward
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveForwardSlow() {
        RFMotor.setPower(0.5*FORWARD_SPEED);
        LFMotor.setPower(0.5*FORWARD_SPEED);
        RBMotor.setPower(0.5*FORWARD_SPEED);
        LBMotor.setPower(0.5*FORWARD_SPEED);
    }

    private void moveBackward() {
        RFMotor.setPower(-FORWARD_SPEED);
        LFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
    }

    private void moveBackwardSlow() {
        RFMotor.setPower(-0.5*FORWARD_SPEED);
        LFMotor.setPower(-0.5*FORWARD_SPEED);
        RBMotor.setPower(-0.5*FORWARD_SPEED);
        LBMotor.setPower(-0.5*FORWARD_SPEED);
    }
    private void moveUpLeftDiagonal() {
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveBackLeftDiagonal() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(-FORWARD_SPEED);
    }

    private void stopRobot() {
        LFMotor.setPower(0);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setPower(0);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setPower(0);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(0);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void moveLeft() {
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveLeftSlow() {
        RFMotor.setPower(0.75*FORWARD_SPEED);
        LFMotor.setPower(-0.75*FORWARD_SPEED);
        RBMotor.setPower(-0.75*FORWARD_SPEED);
        LBMotor.setPower(0.75*FORWARD_SPEED);
    }

    private void moveRight() {
        RFMotor.setPower(-FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
    }

    private void turnRight() {
        LFMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
        RFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
    }


    private void turnLeft() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
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

    private void moveArmDown(){

        servoArm.setPower(FORWARD_SPEED);
    }

    private void moveArmUp(){

        servoArm.setPower(-FORWARD_SPEED);
    }
    private void stopArm() {
        servoArm.setPower(0);
    }
    @Override

    public void runOpMode() {
        connectToHardwareMap();

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.2) {
            moveForward();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.95) {
            turnLeft();

        }

        //stopRobot();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.18) {
            moveRight();
        }

        //stopRobot();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            moveArmDown();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.25){
            moveLeftSlow();
        }

        runtime.reset();
        while (opModeIsActive() && (hsvValues[0] < 200.0 || hsvValues[0] > 220.0 )){
            colorSense();
            moveForwardSlow();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.9) {
            moveForward();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            stopRobot();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.9) {
            moveLeft();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            stopRobot();
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.5){
            moveArmUp();
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            stopRobot();
        }

        runtime.reset();
        do {
            colorSense();
            moveBackwardSlow();
        } while (opModeIsActive() && (hsvValues[0] < 200.0 || hsvValues[0] > 220.0 ));
    }
}