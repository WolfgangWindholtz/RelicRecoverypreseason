package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Robotics on 9/30/2017.
 */
@TeleOp(group = "Claw", name = "Claw")
public class Claw extends OpMode {
    Hardwaremap robot = new Hardwaremap();
    boolean right, left, up, down;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        right = gamepad1.dpad_right;
        left = gamepad1.dpad_left;
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        if (right) {
            robot.servo1.setPosition(1);
        }
        if (left) {
            robot.servo1.setPosition(-1);
        }
        if (up) {
            robot.servo2.setPosition(1);
        }
        if (down) {
            robot.servo2.setPosition(-1);
        }
    }
}