package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 9/29/2017.
 */
@Autonomous(group = "Sensor", name = "Sensor")
public class Color extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();
    double red,blue;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            red = robot.color.red();
            blue = robot.color.blue();
            telemetry.addData("Redness", red);
            telemetry.addData("Blueness",blue);
            telemetry.update();
            if(red>50){
                robot.servo1.setPosition(-1);
                sleep(1000);
            }
            else{
                robot.servo1.setPosition(1);
                sleep(1000);
            }
        }
    }
}
