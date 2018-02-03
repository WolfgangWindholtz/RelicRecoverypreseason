package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "XTele", group = "X")
public class Tele extends OpMode {
    TeleMap bot = new TeleMap();
    double xpow;
    double ypow;
    double zpow;
    double rightx;
    boolean toggle = false;
    double count = 0;

    @Override
    public void init() {
        //initalizes hardware map
        bot.init(hardwareMap);
        bot.mediaPlayer.start();
    }

    public void readGamePad() {
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = gamepad1.left_stick_y;
        xpow = gamepad1.left_stick_x;

        //creates a deadzone for left stick y
        if (Math.abs(ypow) < .05) {
            ypow = 0;

        }
        //creates a deadzone for left stick x
        if (Math.abs(xpow) < .05) {
            xpow = 0;

        }
    }

    @Override
    public void loop() {

        //takes the joystick values and converts to motor speeds through holonomic calculations
        readGamePad();

        double mag = ypow * ypow + xpow * xpow;
        double theta = Math.round(Math.atan2(ypow, xpow) * 4.0 / Math.PI) * Math.PI / 4.0;
        double aPair = mag * Math.cos(theta - Math.PI / 4);
        double bPair = mag * Math.sin(theta - Math.PI / 4);


        //sets movement speeds for motors to move correctly based on joystick input
        //runs at .8 speed to provide driver assisting controls
        bot.motorLF.setPower((bPair - toggle(toggle, zpow)));
        bot.motorRF.setPower((-aPair - toggle(toggle, zpow)));
        bot.motorRB.setPower((-bPair - toggle(toggle, zpow)));
        bot.motorLB.setPower((aPair - toggle(toggle, zpow)));

        //assings the joystick value to another variable
        double slidePower = -gamepad2.left_stick_y;

        if (slidePower > 0) {
            //scales the slidepower to move at a quarter speed
            slidePower /= 4;
        }
        bot.slideMotor.setPower(slidePower);

        if (gamepad1.right_bumper) {
            if (!toggle) {
                toggle = true;
            } else {
                toggle = false;
            }
        }
        telemetry.addData("???",count);
        telemetry.addData("Turtle Mode",toggle);
        telemetry.update();


        //assigns the value of the joystick to a variable
        double relicPower = gamepad2.right_stick_y;

        //sets the variable value to move the motor at the specified speed
        bot.relicMotor.setPower(relicPower);

        if (gamepad2.right_bumper)  //closes the servos to hold the glyph
        {
            gripGlyphTop();
        }
        if (gamepad2.left_bumper) {
            gripGlyphBot();
        }
        if (gamepad2.y) //releases the glyph from the servos
        {
            ram();
        }

        if (gamepad2.x)  //opens the right servo
        {
            openBotWide();
        }
        if (gamepad2.b) {
            releaseGlyphBot();
            releaseGlyphTop();
        }
        if (gamepad2.a) {
            openTopWide();
        }


        if (gamepad2.dpad_left) {
            fingersClose();  // fingers closed for relic
        }
        if (gamepad2.dpad_right) {
            fingersOpen(); // opens finger servo for relic
        }
        if (gamepad2.dpad_up) {
            wristUp();   // brings wrist up for relic
        }
        if (gamepad2.dpad_down) {
            wristDown(); // bring wrist down for relic
        }

    }

    public double toggle(boolean toggle, double power) {
        if (toggle) {
            return power * .4;
        } else {
            return power;
        }

    }

    public void fingersOpen() {
        bot.relicFingers.setPosition(.05);
    }

    public void fingersClose() {
        bot.relicFingers.setPosition(1);
    }

    public void wristUp() {
        bot.relicWrist.setPosition(0);
    }

    public void wristDown() {
        bot.relicWrist.setPosition(.95);
    }

    public void gripGlyphTop(){
        bot.glyphServo3.setPosition(0.27);
        bot.glyphServo4.setPosition(.53);
    }
    public void gripGlyphBot(){
        bot.glyphServo1.setPosition(0.77);
        bot.glyphServo2.setPosition(0.49);
    }

    public void ram() {
        bot.glyphServo1.setPosition(.48);
        bot.glyphServo2.setPosition(.83);
        bot.glyphServo3.setPosition(.87);
        bot.glyphServo4.setPosition(.12);
    }
    public void openBotWide(){
        bot.glyphServo1.setPosition(0.64);
        bot.glyphServo2.setPosition(0.68);
    }
    public void openTopWide(){
        bot.glyphServo3.setPosition(.44);
        bot.glyphServo4.setPosition(0.36);
    }
    public void releaseGlyphBot(){
        bot.glyphServo1.setPosition(0.72);
        bot.glyphServo2.setPosition(0.6);
    }
    public void releaseGlyphTop() {
        bot.glyphServo3.setPosition(.37);
        bot.glyphServo4.setPosition(0.45);
    }
}
