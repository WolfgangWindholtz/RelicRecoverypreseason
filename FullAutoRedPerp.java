package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by WHHS Robotics on 9/23/17.
 */
@Autonomous(name = "FullAutoRedPerp" , group ="Concept")

public class FullAutoRedPerp extends Processor{

    @Override
    public void runOpMode() throws InterruptedException {

        //initalizes hardware on robot
        bot.init(hardwareMap);

        //analyzes the Pictogram image
        waitForStart();

        //once again analyzes the Pictogram image
        checkCol();

        //stores the Pictogram image value in a instance variable
        checkVu();

        //sets servo to grab the glyph touching the robot at the start of autonomous\
        bot.glyphServo4.setPosition(0.5);
        bot.glyphServo3.setPosition(.35);;
        sleep(1000);

        runtime.reset();
        while (runtime.milliseconds() < 400) {
            //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);
        sleep(300);
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(300);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        //moves the robot a distance of 18 inches at an angle of 0 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal
        goAngle(20, 0 );
        sleep(500);
        align(0);
        sleep(500);

        //turns the robot 90 degrees clock wise
        turn(-90);
        sleep(500);
        align(-90);
        sleep(1000);

        //moves the robot a very small increment to line up with the cryptobox
        adjust(33);
        adjust(33);
        adjust(33);
        sleep(1000);
        align(-90);
        sleep(1000);
        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        gotoColumnLeftEnc();

        //stops all motion
        stopBotMotors();
        sleep(1000);

        //releases the glyph and pushes the glyph into the cryptobox
        align(-90);
        sleep(1000);
        score();

    }
}