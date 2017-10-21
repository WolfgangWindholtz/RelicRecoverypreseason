package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Robotics on 10/14/2017.
 */

public abstract class EncoderMove {
    /*ticks per rev is 1120 for Neverest 40 motors
    diameter of omnis is 4 inches
    circumference is 4pi
     */
    Hardwaremap robot = new Hardwaremap();
    public ElapsedTime time = new ElapsedTime();
    Velocity ape;

    public void driveFB(float dist, double power) { ///How far to travel in inches
        float distance = dist / ((float) Math.PI * 4);
        int ticks = Math.round(1120 * distance);
        robot.motorRF.setTargetPosition(ticks);
        robot.motorLF.setTargetPosition(ticks);
        robot.motorLB.setTargetPosition(ticks);
        robot.motorRB.setTargetPosition(ticks);
        robot.motorRF.setPower(power);
        robot.motorLF.setPower(-power);
        robot.motorLB.setPower(-power);
        robot.motorRB.setPower(power);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveRL(float dist, double power) { ///How far to travel in inches
        float distance = dist / ((float) Math.PI * 4);
        int ticks = Math.round(1120 * distance);
        robot.motorRF.setTargetPosition(ticks);
        robot.motorLF.setTargetPosition(ticks);
        robot.motorLB.setTargetPosition(ticks);
        robot.motorRB.setTargetPosition(ticks);
        robot.motorRF.setPower(-power);
        robot.motorLF.setPower(-power);
        robot.motorLB.setPower(power);
        robot.motorRB.setPower(power);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveAngle(double dist, double angle) { /// 2.15 rps
        double x = Math.cos(angle);
        double y = Math.sin(angle);
        double LFpower = .7 * (y - x);
        double RFpower = .7 * (-y - x);
        double RBpower = .7 * (-y + x);
        double LBpower = .7 * (y + x);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ape = robot.imu.getVelocity();
        double velocity = Math.sqrt((ape.xVeloc * ape.xVeloc) + (ape.yVeloc * ape.yVeloc));
        time.reset();
        double seconds = dist/velocity;
        while (time.seconds() <= seconds) {
            robot.motorLF.setPower(.7 * (y - x));
            robot.motorRF.setPower(.7 * (-y - x));
            robot.motorRB.setPower(.7 * (-y + x));
            robot.motorLB.setPower(.7 * (y + x));
        }
    }
}
/*
robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ape = robot.imu.getVelocity();
        time.reset();
        double velocity = Math.sqrt((ape.xVeloc * ape.xVeloc) + (ape.yVeloc * ape.yVeloc));
        while (time.seconds() <= seconds) {
            robot.motorLF.setPower(.7 * (y - x));
            robot.motorRF.setPower(.7 * (-y - x));
            robot.motorRB.setPower(.7 * (-y + x));
            robot.motorLB.setPower(.7 * (y + x));
        }
*/
