package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Thread.sleep;

/**
 * Created by WHHS Robotics on 9/23/17.
 */
@Autonomous(name = "FullAutoBluePerp" , group ="RelicRecovery")

public class FullAutoBluePerp extends Processor{

    @Override
    public void runOpMode() throws InterruptedException {

        //initalizes hardware on robot
        bot.init(hardwareMap);
        //analyzes the Pictogram image
        //checkCol();
        waitForStart();
        //once again analyzes the Pictogram image
        checkCol();

        //stores the Pictogram image value in a instance variable
        checkVu();

        //sets servo to grab the glyph touching the robot at the start of autonomous
        bot.glyphServo1.setPosition(0.53);
        bot.glyphServo2.setPosition(0.5);
        sleep(3000);

        runtime.reset();

        //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }

        bot.slideMotor.setPower(0);

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        //moves the robot a distance of 18 inches at an angle of 180 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal

        goAngle(21,180);
        align(0);

        //turns the robot 90 degrees counter clock wise
        turn(90);

        //moves the robot a very small increment to line up with the cryptobox
        goAngle(.75,0);
        align(90);
        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        gotoColumnRightEnc();

        //stops all motion
        //stopBotMotors();
        sleep(1000);
        //releases the glyph and pushes the glyph into the cryptobox
        align(90);
        score();

    }
}