package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 10/7/2017.
 */
@TeleOp(group = "Claw2", name = "Claw2")
public class Claw2 extends OpMode {
    Hardwaremap robot = new Hardwaremap();
    boolean right, left, up, down;
    double i;

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
            for (i = 0; i < 1; i += .1) {
                if (i > 1) {
                    i = 1;
                }
                robot.servo1.setPosition(i);
            }
        }
        if (left) {
            for (i = 0; i > -1; i -= .1) {
                if (i < -1) {
                    i = -1;
                }
                robot.servo1.setPosition(i);
            }
        }
        if (up) {
            for (i = 0; i < 1; i += .1) {
                if (i > 1) {
                    i = 1;
                }
                robot.servo2.setPosition(i);
            }
        }
        if (down) {
            for (i = 0; i > -1; i -= .1) {
                if (i < -1) {
                    i = -1;
                }
                robot.servo2.setPosition(i);
            }
        }
    }
}
