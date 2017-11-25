package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class hi {
    DcMotor motorBack;
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorFront;

    Servo servo1;
    Servo servo2;
    BNO055IMU imu;


    HardwareMap hwMap;
    public void init(HardwareMap map){
        hwMap = map;

        motorLeft = hwMap.dcMotor.get("motorBL");
        motorRight = hwMap.dcMotor.get("motorFR");
        motorFront = hwMap.dcMotor.get("motorFL");
        motorBack = hwMap.dcMotor.get("motorBR");

        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");
        imu = hwMap.getClass(BNO055IMU,"imu");


    }
}