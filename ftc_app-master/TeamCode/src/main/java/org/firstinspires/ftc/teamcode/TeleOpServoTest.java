package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TeleOpServoTest extends LinearOpMode {

    private Servo servo;
    private Servo moveUp;

@Override

    public void runOpMode() {

    servo = hardwareMap.get(Servo.class, "moveForward");
    moveUp = hardwareMap.get(Servo.class, "moveUp");

        telemetry.addData("Status", "Initalized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive());
        telemetry.addData("Status", "Running");
        telemetry.update();

        if(gamepad2.a) {
            moveForward.setPosition(0.5);
        }

        if(gamepad2.b) {
            moveForward.setPosition(0);
        }



    }
}
