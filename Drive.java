package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 10/6/2017.
 */

public abstract class Drive extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();
    public void driveFB(double power){
        robot.motorRF.setPower(power);
        robot.motorLF.setPower(-power);
        robot.motorLB.setPower(-power);
        robot.motorRB.setPower(power);
    }
    public void driveRL(double power){
        robot.motorRF.setPower(-power);
        robot.motorLF.setPower(-power);
        robot.motorLB.setPower(power);
        robot.motorRB.setPower(power);
    }
    public void strafeRFLB(double power){
        robot.motorLF.setPower(-power);
        robot.motorRB.setPower(power);
    }
    public void strafeLFRB(double power){
        robot.motorRF.setPower(power);
        robot.motorLB.setPower(-power);
    }
}
