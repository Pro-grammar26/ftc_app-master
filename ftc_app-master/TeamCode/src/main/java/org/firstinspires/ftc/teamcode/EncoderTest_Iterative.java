package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;





@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")



public class EncoderTest_Iterative extends OpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveF = null;
    private DcMotor rightDriveF = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveB = null;
    private double target = 0;
    private double ticksPerRotation = 1120/1.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */


    private void goToPosition (double targetPosition) {
        double difference = targetPosition - rightDriveF.getCurrentPosition();
        telemetry.addData("difference", difference);
        setTankPower(-difference/500);
    }

    private void setTankPower (double input) {
            double power = Range.clip(input, -0.5, 5);
        telemetry.addData("power", power);
        rightDriveF.setPower(power);
        leftDriveF.setPower(power);
        rightDriveB.setPower(power);
        leftDriveB.setPower(power);
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveF  = hardwareMap.get(DcMotor.class, "LFMotor");
        rightDriveF = hardwareMap.get(DcMotor.class, "RFMotor");
        leftDriveB  = hardwareMap.get(DcMotor.class, "LBMotor");
        rightDriveB = hardwareMap.get(DcMotor.class, "RBMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);

        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        target = 0;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("cool", leftDriveF.getCurrentPosition());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    boolean triggered = false;
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        if(gamepad1.x && triggered == false) {
            target = target + ticksPerRotation;
            triggered = true;
        }
        if(gamepad1.y) {
            triggered = false;
        }
        //leftDriveB.setPower(1.0);
        //rightDriveB.setPower(1.0);
        goToPosition(target);
        telemetry.addData("cool", target);
        telemetry.addData("value", rightDriveF.getCurrentPosition());
        telemetry.addData("value", leftDriveF.getCurrentPosition());

        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
