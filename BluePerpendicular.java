package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/16/2017.
 */
@Autonomous(name = "BluePerpendicular", group = "jfjf")
public class BluePerpendicular extends Processor{
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

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        goAnglePower(20,180,.4);
        sleep(500);
        align(0);
        sleep(500);
        turn(90);
        align(90);
        goAnglePower(1.2,0,.3);
        raiseColorServo();
        adjust(.047);
        sleep(200);
        adjust(.047);
        adjust(.047);
        align(90);
        gotoColumnRight();

        stopBotMotors();


        bot.colorServo.setPosition(0);
        sleep(500);
        align(90);
        align(90);
        align(90);
        //releases the glyph and pushes the glyph into the cryptobox
        score1(90);
        stopBotMotors();
    }
}