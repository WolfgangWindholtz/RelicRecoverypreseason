package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 10/28/2017.
 */
@Autonomous(group = "Testis",name = "test")
public class test extends Processum {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double power = .7;
            robot.motorRF.setPower(power);
            robot.motorLF.setPower(-power);
            robot.motorLB.setPower(-power);
            robot.motorRB.setPower(power);
        }
    }
}
