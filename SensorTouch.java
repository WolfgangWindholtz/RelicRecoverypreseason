package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Robotics on 12/15/2017.
 */
@Autonomous(name = "touch" , group ="Concept")

public class SensorTouch extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        //analyzes the Pictogram image
        waitForStart();
        while (opModeIsActive()){
            if(bot.touchSensor.isPressed()) {
                telemetry.addData("pressed", bot.touchSensor.isPressed());
                telemetry.update();
            }
        }
    }
}
