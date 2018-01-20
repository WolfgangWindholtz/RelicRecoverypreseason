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
        sekrit_code();

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
            openArms();
        }
        if (gamepad2.b) {
            releaseGlyphBot();
        }
        if (gamepad2.a) {
            releaseGlyphTop();
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
        bot.relicFingers.setPosition(0);
    }

    public void fingersClose() {
        bot.relicFingers.setPosition(1);
    }

    public void wristUp() {
        bot.relicWrist.setPosition(0);
    }

    public void wristDown() {
        bot.relicWrist.setPosition(1);
    }

    public void gripGlyphTop(){
        bot.glyphServo3.setPosition(0.20);
        bot.glyphServo4.setPosition(.59);
    }
    public void gripGlyphBot(){
        bot.glyphServo1.setPosition(0.55);
        bot.glyphServo2.setPosition(0.45);
    }

    public void ram() {
        bot.glyphServo1.setPosition(.15);
        bot.glyphServo2.setPosition(.85);
        bot.glyphServo3.setPosition(.87);
        bot.glyphServo4.setPosition(.12);
    }
    public void openArms(){
        bot.glyphServo1.setPosition(0.28);
        bot.glyphServo2.setPosition(0.72);
        bot.glyphServo3.setPosition(.44);
        bot.glyphServo4.setPosition(0.34);
    }
    public void releaseGlyphBot(){
        bot.glyphServo1.setPosition(0.4);
        bot.glyphServo2.setPosition(0.6);
    }
    public void releaseGlyphTop(){
        bot.glyphServo3.setPosition(.32);
        bot.glyphServo4.setPosition(0.46);
    }

    public void sekrit_code() {
        if(count==0) {
            if (gamepad1.dpad_up) {
                count++;
            }
        }
        if (count == 1) {
            if (gamepad1.dpad_up) {
                count++;
            }
        }
        if (count == 2) {
            if (gamepad1.dpad_down) {
                count++;
            }
        }
        if (count == 3) {
            if (gamepad1.dpad_down) {
                count++;
            }
        }
        if (count == 4) {
            if (gamepad1.dpad_left) {
                count++;
            }
        }
        if (count == 5) {
            if (gamepad1.dpad_right) {
                count++;
            }
        }
        if (count == 6) {
            if (gamepad1.dpad_left) {
                count++;
            }
        }
        if (count == 7) {
            if (gamepad1.dpad_right) {
                count++;
            }
        }
        if (count == 8) {
            if (gamepad1.b) {
                count++;
            }
        }
        if(count==9){
            if (gamepad1.a) {
                count++;
            }
        }
        if(count==10){
            if(gamepad1.start){
                bot.mediaPlayer.stop();
            }
        }
    }
}
