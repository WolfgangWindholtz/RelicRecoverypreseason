package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by WHHS robotics on 9/23/17.
 */
@Autonomous(name = "FullAutoRedParallel" , group ="Concept")

public class FullAutoRedParallel extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        //initializes all hardware on robot
        bot.init(hardwareMap);

        //analyzes the Pictogram image
        waitForStart();

        //analyzes the Pictogram image
        checkCol();

        //stores the Pictogram image value in a instance variable
        checkVu();


        //sets servo to grab the glyph touching the robot at the start of autonomous
        bot.glyphServo4.setPosition(0.5);
        bot.glyphServo3.setPosition(.35);;
        sleep(1500);

        runtime.reset();
        while (runtime.milliseconds() < 400) {
            //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(700);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        //moves the robot a distance of 30 inches at an angle of 0 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal
        goAngle(20, 0);
        sleep(1000);
        align(0);
        sleep(1000);


        //turns the robot 180 degrees counter clock wise
        turn(-180);

        //moves the robot a very small increment to line up with the cryptobox
        sleep(2000);
        align(180);
        sleep(1000);
        adjust(35);
        adjust(35);
        adjust(35);
        sleep(1000);
        align(180);
        sleep(1000);

        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        gotoColumnLeftEnc();

        //stops all motion
        stopBotMotors();
        sleep(1000);

        //releases the glyph and pushes the glyph into the cryptobox
        align(180);
        sleep(1000);
        score();
    }
}