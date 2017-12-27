package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by WHHS robotics on 9/23/17.
 */
@Autonomous(name = "TestAuto" , group ="Concept")

public class TestAuto extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        //initializes all hardware on robot
        bot.init(hardwareMap);
        double count = 0;
        //analyzes the Pictogram image
        waitForStart();
        adjust(33.75);
        adjust(33.75);
        adjust(33.75);
        sleep(500);
        align(0);
        sleep(1000);
        goColumnL(2);
    }
}
