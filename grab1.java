package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by khadija on 9/16/2017.
 */
@Autonomous(name="grab1",group="grab1")
public class grab1 extends LinearOpMode {

    Servo servo1 = null;
    Servo servo2 = null;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("servo1's direction %s",servo1.getDirection());
            telemetry.addData("servo2's direction: %s",servo2.getDirection());

            servo1.setPosition(1);


            servo1.close();
            servo2.close();

        }

    }
}