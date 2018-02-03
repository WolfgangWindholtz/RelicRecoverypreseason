package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/15/2017.
 */
@Autonomous(name = "BlueParallel", group = "fjfrjkdk")
public class BlueParallel extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        checkCol();
        grabGlyph();
        align(0);

        //knocks the correct jewel off according to our alliance colo

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        goAnglePower(24,160,.8);
        align(0);
        turn(180);
        align(180);
        raiseColorServo();
        adjust(.047);
        sleep(200);
        adjust(.047);
        adjust(.047);
        align(180);


        gotoColumnRight();

        stopBotMotors();

        bot.colorServo.setPosition(0);

        sleep(500);
        align(180);
        //driveToDistance();
        score1(180);
        stopBotMotors();
    }
}