package org.firstinspires.ftc.teamcode;

/**
 * Created by Robotics on 9/16/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "Servo",name = "Servo")
public class RevServo extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            robot.servo1.setPosition(1);
        }
    }
}
