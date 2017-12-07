package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        ;

    }
}