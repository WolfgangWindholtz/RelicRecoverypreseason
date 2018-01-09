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
        //analyzes the Pictogram image
        waitForStart();
        adjust(14.3);
        adjust(14.3);
        adjust(14.3);
    }
}
