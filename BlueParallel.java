package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/15/2017.
 */
@Autonomous(name = "BlueParallel", group = "fjfrjkdk")
public class BlueParallel extends Processor{

    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        checkVu();
        checkCol();
        grabGlyph();

        //knocks the correct jewel off according to our alliance colo

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        goAnglePower(24,160,.4);
        sleep(500);
        align(0);
        sleep(500);
        turn(180);
        align(180);
        goAnglePower(2,180,.3);
        raiseColorServo();
        adjust(.048);
        sleep(200);
        adjust(.048);
        adjust(.048);
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