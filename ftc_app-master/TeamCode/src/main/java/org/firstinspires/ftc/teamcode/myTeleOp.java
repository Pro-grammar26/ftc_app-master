package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FormulaRTeleOp extends LinearOpMode {

    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private DcMotor slidePlacer;
    private CRServo servoArm;
    private CRServo grabbingServo;

    private void runIntake(){
        double intakePower = 0;
        intakePower = this.gamepad1.left_trigger;
        if (intakePower >= 0.1 && intakePower <= 0.5){
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        } else if (intakePower >= 0.5){
            IntakeL.setPower(-0.9);
            IntakeR.setPower(0.9);
        }
        telemetry.addData("Intake Motor Power", IntakeR.getPower());
        telemetry.addData("Intake Motor Power", IntakeL.getPower());
    }

    private void runOutake(){
        double outakePower = 0;
        outakePower = this.gamepad1.right_trigger;
        if (outakePower >= 0.1 && outakePower <= 0.5){
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        } else if (outakePower >= 0.5){
            IntakeL.setPower(1);
            IntakeR.setPower(-1);
        }
        telemetry.addData("OutakePower", outakePower);
        telemetry.addData("Outake Motor Power", IntakeR.getPower());
        telemetry.addData("Outake Motor Power", IntakeL.getPower());
    }
    private void runServo(){
        double armPower = 0;
        armPower = -this.gamepad2.left_stick_x;
        servoArm.setPower(1.25*armPower);
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Servo Power", servoArm.getPower());
        telemetry.update();
    }
    private void moveDriveTrain(){
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;
        RFMotor.setPower(pivot + (-vertical + horizontal));
        RBMotor.setPower(pivot + -vertical - horizontal);
        LFMotor.setPower(-pivot + -vertical - horizontal);
        LBMotor.setPower(-pivot + (-vertical + horizontal));
    }

    @Override
    public void runOpMode() {
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        servoArm = hardwareMap.get(CRServo.class, "servoArm");


        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            moveDriveTrain();
            runIntake();
            runOutake();
            runServo();
            // Put loop blocks here.
            telemetry.update();

        }
    }
}