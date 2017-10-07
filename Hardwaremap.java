package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Robotics on 9/22/2017.
 */
public class Hardwaremap{
    public DcMotor motorLF;
    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor motorLB;
    public Servo servo1;
    public Servo servo2;
    public ColorSensor sensor1;
    public HardwareMap hwMap = null;

    public void init(HardwareMap ahwmap) {
        hwMap = ahwmap;
        motorLF = hwMap.dcMotor.get("motorLF");
        motorRF = hwMap.dcMotor.get("motorRF");
        motorRB = hwMap.dcMotor.get("motorRB");
        motorLB = hwMap.dcMotor.get("motorLB");
        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");
        sensor1 = hwMap.colorSensor.get("sensor1");
        motorLF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLB.setDirection(DcMotorSimple.Direction.FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
    }
}
