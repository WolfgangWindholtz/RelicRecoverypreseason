package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 10/7/2017.
 */
@TeleOp(group = "XDrive2", name = "XDrive2")
public class XDrive2 extends OpMode {
    Hardwaremap robot = new Hardwaremap();
    double x,y,z;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        x = gamepad1.left_stick_x/2;
        y = gamepad1.left_stick_y/2;
        z = gamepad1.right_stick_x;
        robot.motorLF.setPower(y-x-z);
        robot.motorRF.setPower(-y-x-z);
        robot.motorRB.setPower(-y+x-z);
        robot.motorLB.setPower(y+x-z);
    }
}
