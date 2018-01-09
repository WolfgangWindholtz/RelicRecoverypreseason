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
    float x, y, w;
    static double deadzone = .5;

    @Override
    public void init() {

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        w = gamepad1.right_stick_x;
        if (Math.abs(y) > deadzone) { ///Up and Down
            robot.motorRF.setPower(Range.clip(y, -1, 1));
            robot.motorLF.setPower(Range.clip(-y, -1, 1));
            robot.motorLB.setPower(Range.clip(-y, -1, 1));
            robot.motorRB.setPower(Range.clip(y, -1, 1));
        }
        if (Math.abs(x) > deadzone) { ///Right and Left Strafe
            robot.motorRF.setPower(Range.clip(-x, -1, 1));
            robot.motorLF.setPower(Range.clip(-x, -1, 1));
            robot.motorLB.setPower(Range.clip(x, -1, 1));
            robot.motorRB.setPower(Range.clip(x, -1, 1));
        }
        if (Math.abs(w) > deadzone) { ///Turning Right and Left
            robot.motorRF.setPower(Range.clip(-w, -1, 1));
            robot.motorLF.setPower(Range.clip(-w, -1, 1));
            robot.motorLB.setPower(Range.clip(-w, -1, 1));
            robot.motorRB.setPower(Range.clip(-w, -1, 1));
        }
        if (gamepad1.dpad_up == true){
            robot.motorLF.setPower(-1);
            robot.motorRB.setPower(1);
        }
        if (gamepad1.dpad_down == true){
            robot.motorLF.setPower(1);
            robot.motorRB.setPower(-1);
        }
        if (gamepad1.dpad_right == true){
            robot.motorRF.setPower(-1);
            robot.motorLB.setPower(1);
        }
        if (gamepad1.dpad_left == true){
            robot.motorRF.setPower(1);
            robot.motorLB.setPower(-1);
        }
        else {
            robot.motorRF.setPower(0);
            robot.motorLF.setPower(0);
            robot.motorLB.setPower(0);
            robot.motorRB.setPower(0);
        }
    }
}