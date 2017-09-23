package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 9/16/2017.
 */

@TeleOp(group = "XDrive",name = "XDrive")
public class XDrive extends OpMode {
    Hardwaremap robot = new Hardwaremap();
    static float x, y, w, z;
    static double deadzone;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        w = gamepad1.right_stick_x;
        z = gamepad2.right_stick_y;
        if (Math.abs(y) > deadzone) {
            robot.motor1.setPower(Range.clip(y, -1, 1));
            robot.motor2.setPower(Range.clip(y, -1, 1));
            robot.motor3.setPower(Range.clip(y, -1, 1));
            robot.motor4.setPower(Range.clip(y, -1, 1));
        }
        if (Math.abs(x) > deadzone) {
            robot.motor1.setPower(Range.clip(x, -1, 1));
            robot.motor2.setPower(Range.clip(-x, -1, 1));
            robot.motor3.setPower(Range.clip(x, -1, 1));
            robot.motor4.setPower(Range.clip(-x, -1, 1));
        }
        if (Math.abs(w) > deadzone) {
            robot.motor1.setPower(Range.clip(w, -1, 1));
            robot.motor2.setPower(Range.clip(-w, -1, 1));
            robot.motor3.setPower(Range.clip(w, -1, 1));
            robot.motor4.setPower(Range.clip(-w, -1, 1));
        }
        if (Math.abs(z) > deadzone) {
            robot.motor1.setPower(Range.clip(-z, -1, 1));
            robot.motor2.setPower(Range.clip(z, -1, 1));
            robot.motor3.setPower(Range.clip(-z, -1, 1));
            robot.motor4.setPower(Range.clip(z, -1, 1));
        }
    }
}