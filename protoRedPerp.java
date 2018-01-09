package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Robotics on 12/16/2017.
 */
@Autonomous(name = "protoRedPerp" , group ="proto")
public class protoRedPerp extends protoProcessor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        //analyzes the Pictogram image
        waitForStart();

        //once again analyzes the Pictogram image
        checkCol();

        //stores the Pictogram image value in a instance variable
        checkVu();

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        //moves the robot a distance of 18 inches at an angle of 0 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal
        goAngle(21, 0 );
        sleep(1000);

        //turns the robot 90 degrees clock wise
        turn(-90);
        sleep(500);

        //moves the robot a very small increment to line up with the cryptobox
        goAngle(.75,180);

        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        gotoColumnLeftEnc();

        //stops all motion
        stopBotMotors();
        sleep(1000);

        //releases the glyph and pushes the glyph into the cryptobox
        score();

    }
}
