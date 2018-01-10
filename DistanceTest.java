package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Walnut Robotics on 1/9/2018.
 */
@Autonomous(name = "distance", group = "fjfj")
public class DistanceTest extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("RangeSensor(CM): ",bot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("ProximitySensor(CM)", bot.colorSensor2.getDistance(DistanceUnit.CM));
            telemetry.addData("MaxBotics(In)", getDistanceLeft());
            telemetry.addData("Voltage ", bot.ultrasonicLeft.getVoltage());

            telemetry.update();
        }
    }

}
