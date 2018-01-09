package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 12/15/2017.
 */
@TeleOp(group = "proto",name = "protoTele")
public class protoTeleOp extends OpMode{
    protoTeleMap bot = new protoTeleMap();
    double xpow;
    double ypow;
    double zpow;
    double x,y,z;
    double rightx;
    boolean toggle = false;

    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //https://www.vexforum.com/index.php/7873-programming-mecanum-wheels/0
        zpow = gamepad1.right_stick_x;
        ypow = gamepad1.left_stick_y;
        xpow = gamepad1.left_stick_x;

        bot.motorLF.setPower(.7*(-ypow-xpow-toggle(toggle,zpow)));
        bot.motorRF.setPower(.7*(ypow-xpow-toggle(toggle,zpow)));
        bot.motorRB.setPower(.7*(ypow+xpow-toggle(toggle,zpow)));
        bot.motorLB.setPower(.7*(-ypow+xpow-toggle(toggle,zpow)));

        if(gamepad1.dpad_left){
            bot.motorLF.setPower(.7);
            bot.motorRF.setPower(.7);
            bot.motorRB.setPower(-.7);
            bot.motorLB.setPower(-.7);
        }
        if(gamepad1.dpad_right){
            bot.motorLF.setPower(-.7);
            bot.motorRF.setPower(-.7);
            bot.motorRB.setPower(.7);
            bot.motorLB.setPower(.7);
        }
        if(gamepad1.dpad_up){
            bot.motorLF.setPower(-.7);
            bot.motorRF.setPower(.7);
            bot.motorRB.setPower(.7);
            bot.motorLB.setPower(-.7);
        }
        if(gamepad1.dpad_down){
            bot.motorLF.setPower(.7);
            bot.motorRF.setPower(-.7);
            bot.motorRB.setPower(-.7);
            bot.motorLB.setPower(.7);
        }
        else{
            bot.motorLF.setPower(0);
            bot.motorRF.setPower(0);
            bot.motorRB.setPower(0);
            bot.motorLB.setPower(0);
        }

        double slidePower = -gamepad2.left_stick_y;

        if(slidePower>0)
        {
            slidePower /= 4;
        }

        bot.slideMotor.setPower(slidePower);

        if(gamepad1.a){
            if(!toggle){
                toggle = true;
            }
            else {
                toggle = false;
            }

        }


        double relicPower = gamepad2.right_stick_y;
        bot.relicMotor.setPower(relicPower);

        if(gamepad2.a)  // gripGlyphs
        {
            intakeOut();
        }
        if(gamepad2.x)  // openLeft
        {
            intakeIn();
        }
        if(gamepad2.b)  // openRight
        {
            intakeOff();
        }
        if(gamepad2.right_bumper){
            dump();
        }
        if(gamepad2.left_bumper){
            load();
        }
        if(gamepad2.dpad_left){
            fingersClose();  // fingers closed for relic
        }
        if(gamepad2.dpad_right){
            fingersOpen(); // opens finger servo for relic
        }
        if(gamepad2.dpad_up){
            wristUp();   // brings wrist up for relic
        }
        if(gamepad2.dpad_down){
            wristDown(); // bring wrist down for relic
        }

    }

    public void fingersOpen(){
        bot.relicFingers.setPosition(.6);
    }

    public void fingersClose(){
        bot.relicFingers.setPosition(.95);
    }

    public void wristUp() {
        bot.relicWrist.setPosition(.7);
    }

    public void wristDown() {
        bot.relicWrist.setPosition(0);
    }
    public void dump() {
        bot.dumpServo2.setPosition(.5);
        bot.dumpServo1.setPosition(.5);
    }
    public void load() {
        bot.dumpServo2.setPosition(.03);
        bot.dumpServo1.setPosition(.9);
    }
    public void intakeIn() {
        bot.intakeMotor.setPower(.5);
    }
    public void intakeOut() {
        bot.intakeMotor.setPower(-.5);
    }
    public void intakeOff() {
        bot.intakeMotor.setPower(0);
    }

    public double toggle(boolean toggle, double power){
        if(toggle){
            return power * .4;
        }
        else{
            return power;
        }

    }
}
