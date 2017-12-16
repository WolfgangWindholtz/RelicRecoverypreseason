package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@TeleOp(name = "teleleleleleleleop", group = "teleop")
public class teleop extends OpMode{
    telemap bot = new telemap();
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

    public void readGamePad() {
        zpow = gamepad1.right_stick_x;//direction not actually
        ypow = gamepad1.left_stick_y;// variable names are incoorect
        xpow = gamepad1.left_stick_x;
    }

    @Override
    public void loop() {
        readGamePad();

        double mag = Math.sqrt(ypow * ypow + xpow * xpow);
        double theta = Math.atan2(ypow, xpow);
        double aPair = mag * Math.cos(theta-Math.PI/4);
        double bPair = mag * Math.sin(theta-Math.PI/4);


        bot.motorLF.setPower(.7*(bPair-toggle(toggle,zpow)));
        bot.motorRF.setPower(.7*(-aPair-toggle(toggle,zpow)));
        bot.motorRB.setPower(.7*(-bPair-toggle(toggle,zpow)));
        bot.motorLB.setPower(.7*(aPair-toggle(toggle,zpow)));

        /*
        x = gamepad1.left_stick_x/2;
        y = gamepad1.left_stick_y/2;
        z = gamepad1.right_stick_x;
        bot.motorRF.setPower(y-x-z);
        bot.motorLF.setPower(-y-x-z);
        bot.motorLB.setPower(-y+x-z);
        bot.motorRB.setPower(y+x-z);
        */

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
            closeTop();
        }
        if(gamepad2.x)  // openLeft
        {
            closeBot();
        }
        if(gamepad2.b)  // openRight
        {
            ram();
        }
        if(gamepad2.y) // releaseGlyphs
        {
            openTop();
        }
        if(gamepad2.right_bumper){
            releaseGlyph();
        }
        if(gamepad2.left_bumper){
            gripGlyph();
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

    public void ram(){
        bot.glyphServo4.setPosition(.95);
        bot.glyphServo3.setPosition(.1);
        bot.glyphServo2.setPosition(.03);
        bot.glyphServo1.setPosition(.9);
        //telemetry.addData("RAM","Random Access Memory");
    }

    public void openTop(){
        bot.glyphServo4.setPosition(0.5);
        bot.glyphServo3.setPosition(.35);
    }
    public void openBot(){
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
    }
    public void gripGlyph() {
        closeBot();
        closeTop();
    }
    public void closeTop(){
        bot.glyphServo4.setPosition(0.35);
        bot.glyphServo3.setPosition(.5);
    }
    public void closeBot(){
        bot.glyphServo1.setPosition(0.53);
        bot.glyphServo2.setPosition(0.5);

    }
    public void openRight() {
        bot.glyphServo1.setPosition(0.53);
        bot.glyphServo4.setPosition(0.35);
    }

    public void openLeft() {
        bot.glyphServo2.setPosition(0.5);
        bot.glyphServo3.setPosition(.5);
    }

    public void releaseGlyph() {
        openTop();
        openBot();
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