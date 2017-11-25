package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by khadija on 10/13/2017.
 */
@TeleOp(name="claw",group="claw")
public class claw extends LinearOpMode {
    Servo servo1;
    Servo servo2;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("servo1's direction %s",servo1.getDirection());
            telemetry.addData("servo2's direction: %s",servo2.getDirection());
            if (gamepad1.a){
                servo1.setPosition(1);
                if(gamepad1.a){
                    servo1.setPosition(-1);
                }

            }
            else if (gamepad1.b){
                servo2.setPosition(1);
                if (gamepad1.a){
                    servo2.setPosition(-1);
                }

            }


            servo1.close();
            servo2.close();


        }

    }
}
