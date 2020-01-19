package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class ModulerTeleOp extends LinearOpMode {

        private DcMotor RFMotor;
        private DcMotor RBMotor;
        private DcMotor LFMotor;
        private DcMotor LBMotor;
        private DcMotor RIMotor;
        private DcMotor LIMotor;

        void initialize() {
                RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
                RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
                LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
                LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
                RIMotor = hardwareMap.get(DcMotor.class, "IntakeLeft");
                LIMotor = hardwareMap.get(DcMotor.class, "IntakeRight");
                //Sending status to drivers station
                telemetry.addData("Status", "Initialized");
                telemetry.update();
        }

        void moveForwardAndBackwards() {
                setMotorPower(RFMotor,-gamepad1.left_stick_y, "RFMotor");
                setMotorPower(RBMotor,-gamepad1.left_stick_y, "RBMotor");
                setMotorPower(LFMotor,-gamepad1.left_stick_y, "LFMotor");
                setMotorPower(LBMotor,-gamepad1.left_stick_y, "LBMotor");


        }

        void slideLeftAndRight() {

                setMotorPower(RFMotor,-gamepad1.left_stick_x, "RFMotor");
                setMotorPower(RBMotor,gamepad1.left_stick_x, "RBMotor");
                setMotorPower(LFMotor,gamepad1.left_stick_x, "LFMotor");
                setMotorPower(LBMotor,-gamepad1.left_stick_x, "LBMotor");


        }


        void turnRightAndLeft() {
                setMotorPower(RFMotor,-gamepad1.right_stick_x, "RFMotor");
                setMotorPower(RBMotor,gamepad1.right_stick_x, "RBMotor");
                setMotorPower(LFMotor,-gamepad1.right_stick_x, "LFMotor");
                setMotorPower(LBMotor,gamepad1.right_stick_x, "LBMotor");

        }

        void moveUpLeftAndDownRightDiagnal() {
                setMotorPower(RFMotor, gamepad1.right_stick_x, "RFMotor");
                setMotorPower(RBMotor, gamepad1.right_stick_x, "RFMotor");

        }

        void Intake() {
                setMotorPower(RIMotor,gamepad1.right_trigger, "RIMotor");
                setMotorPower(LIMotor,gamepad1.right_trigger, "LIMotor");

        }
        void Outake() {
                setMotorPower(RIMotor,-gamepad1.right_trigger, "RIMotor");
                setMotorPower(LIMotor,-gamepad1.right_trigger,"LIMotor");

        }


        void setMotorPower(DcMotor motor, double targetPower, String motorName) {
                motor.setPower(targetPower);
                if (targetPower == 0) {
                        //Set the code to brake the motor

                }
                telemetry.addData("Target Power for %s motor is %.3f",motorName, targetPower);
                telemetry.addData("Actual Power for %s motor is %.3f",motorName, motor.getPower());
                assert(targetPower == motoro.getPower());
                telemetry.update();
        }

        @Override
        public void runOpMode() {

                initialize();
                waitForStart();
                while (opModeIsActive()) {
                        moveForwardAndBackwards();
                        slideLeftAndRight();
                        turnRightAndLeft();
                        Intake();
                        Outake();



                }

        }
}
