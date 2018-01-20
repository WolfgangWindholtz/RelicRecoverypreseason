package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 12/7/2017.
 */
@TeleOp(group = "Concept",name = "servotest")
public class test extends OpMode {
    TeleMap bot = new TeleMap();

    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            bot.glyphServo1.setPosition(.5);
        }
        if(gamepad1.b){
            bot.glyphServo2.setPosition(.5);
        }
        if(gamepad1.x){
            bot.glyphServo3.setPosition(.5);
        }
        if(gamepad1.y){
            bot.glyphServo4.setPosition(.5);
        }
    }
}
