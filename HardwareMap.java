package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by khadija on 9/2/2017.
 */
public abstract class HardwareMap extends LinearOpMode {
    SRServo servo = null;

    public void main() throws InterruptedException {

    servo = hardwareMap.servo.get("servo");
        


    }
}