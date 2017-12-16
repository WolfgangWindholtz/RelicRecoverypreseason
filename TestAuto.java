package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by WHHS robotics on 9/23/17.
 */
@Autonomous(name = "TEST" , group ="Concept")

public class TestAuto extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        //initializes all hardware on robot
        bot.init(hardwareMap);
        //analyzes the Pictogram image
        waitForStart();
        //analyzes the Pictogram image
        if(opModeIsActive()) {
            goAngle(10,0);
            goAngle(10,180);
            goAngle(10,90);
            goAngle(10,270);
        }
    }
}