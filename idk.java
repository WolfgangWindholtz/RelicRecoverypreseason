package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by khadija on 10/27/2017.
 */
@TeleOp(name="idk",group = "idk")
public class idk extends OpMode {
    DcMotor motorBack;
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorFront;

    Servo servo1;
    Servo servo2;
    BNO055IMU imu;


    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorBL");
        motorRight = hardwareMap.dcMotor.get("motorFR");
        motorFront = hardwareMap.dcMotor.get("motorFL");
        motorBack = hardwareMap.dcMotor.get("motorBR");

        servo1 = hardwareMap.servo.get("servo1");//wrist
        servo2 = hardwareMap.servo.get("servo2");//fingers
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
    servo1.setPosition(1);
    }
    @Override
    public void loop() {
            if (gamepad1.a) {
                servo1.setPosition(.5);//wrist up
            }
            else if (gamepad1.b){
                servo1.setPosition(.75);//wrist down
            }
            if (gamepad1.x) {
                servo2.setPosition(0);//open the fingers
            }
            else if (gamepad1.y) {
                    servo2.setPosition(.95);//close the fingers
            }
    }

    @Override
    public void stop() {
    }
}