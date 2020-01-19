package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
    public class StoneAutonomous extends LinearOpMode {

        DcMotor RfMotor;
        DcMotor RbMotor;
        DcMotor LfMotor;
        DcMotor LbMotor;
        Servo pickerUpper;
        ColorSensor colorSensor;

        private static final String VUFORIA_KEY =
                " AVSewev/////AAABmX8I2SokYUBRtwZFqWwf2S0L/1U0H2hghsH/hcbE4M75kOBoyKRC0jY9+Z1Wl67G3BvXFg+bI0pgQf7ToTkp0v/AZ0M3MnuHgMNghA+lGj9WjwoQYcUbnzoTab5CckWAQj+CF+eFOV5fKsZy3XYZ3fB6Z4tAyQsH07V2324bQI8pbXPlVzq4EykXtcIQYRmURoXXe4j87WtRALDQxrSo/nxOIEg38Yq79In/6gMZelTzogTWEpwx4pWhb8QiiexmedrIghylOPVu6B8TOgqPEZjYmX74ZvYfkn7t7+0nQcgBxz3nXGZTV2xQts57u52ZY3L7fkBNYFDMj7GpsB2eNKIJLXKclLD6cqIBLf5augZv ";

        @Override
        public void runOpMode() {
            hardwareMap.dcMotor.get("RfMotor");
            hardwareMap.dcMotor.get("RbMotor");
            hardwareMap.dcMotor.get("LfMotor");
            hardwareMap.dcMotor.get("LbMotor");
            hardwareMap.servo.get("pickerUpper");
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

            RfMotor.setDirection(DcMotor.Direction.REVERSE);
            RbMotor.setDirection(DcMotor.Direction.REVERSE);

            pickerUpper.setPosition(0);



            waitForStart();
            RfMotor.setPower(1);
            RbMotor.setPower(1);
            LfMotor.setPower(1);
            LbMotor.setPower(1);

            sleep(2000);

            RfMotor.setPower(1);
            RbMotor.setPower(1);
            LfMotor.setPower(-1);
            LbMotor.setPower(-1);

            sleep(250);

            //now use the tensorFlow model to find each skyStone

            pickerUpper.setPosition(1);

            sleep(1000);

            RfMotor.setPower(-1);
            RbMotor.setPower(-1);
            LfMotor.setPower(-1);
            LbMotor.setPower(-1);

            sleep(1000);

                RfMotor.setPower(-1);
                RbMotor.setPower(-1);
                LfMotor.setPower(1);
                LbMotor.setPower(1);
                while(opModeIsActive()){
                    telemetry.addData("Status", "Running");
                    telemetry.update();
                }

//         use colorSensor to stop under the bridge so we know where we are on the field


        }
    }

