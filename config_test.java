package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Robotics on 11/22/2017.
 */
@Autonomous(group = "config_test",name = "config_test")
public class config_test extends Drive{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            driveFB(.7);
        }
    }
}
