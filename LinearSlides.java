package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 10/7/2017.
 */
@TeleOp(group = "Linear",name = "Linear")
public class LinearSlides extends OpMode {
    Hardwaremap robot = new Hardwaremap();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.motorLF.setTargetPosition(1222);
            robot.motorLF.setPower(.7);
            robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (gamepad1.b) {
            robot.motorLF.setTargetPosition(1222);
            robot.motorLF.setPower(-.7);
            robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
