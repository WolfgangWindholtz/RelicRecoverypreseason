package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by khadija on 10/27/2017.
 */

public class ridges extends autopid2 {


    public void runOpMode(){
    waitForStart();

        while(opModeIsActive()){
            DriveLeft(1,5);
            DriveForward(1,3);
        }
     }
}
