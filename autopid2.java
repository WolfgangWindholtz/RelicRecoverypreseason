package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class autopid2 extends LinearOpMode {
    hi robot = new hi();
    double motorRevolutionTicks = 1120;

    public void TurnTicks(double power, int distance, DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(distance);
        motor.setPower(power);
        while(motor.isBusy()&&opModeIsActive()){
            // wait for motor to reach position
        }
        motor.setPower(0);
    }
    public void SetPower(double left, double back, double front, double right){
        robot.motorLeft.setPower(left);
        robot.motorBack.setPower(back);
        robot.motorFront.setPower(front);
        robot.motorRight.setPower(right);
    }
    public void StopDriving(){
        SetPower(0,0,0,0);
    }
    public void DriveForwardLeft(double power, int distance){
        robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition()-distance);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetPower(-power,0,0,power);
        while(opModeIsActive()&&robot.motorLeft.isBusy()){
            // wait for motor to reach position
        }
        StopDriving();
    }
    public void DriveBackwardRight(double power, int distance){DriveForwardLeft(-power, -distance);}
    public void DriveForwardRight(double power, int distance){
        robot.motorBack.setTargetPosition(robot.motorBack.getCurrentPosition()+distance);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetPower(0,power,-power,0);
        while(opModeIsActive()&&robot.motorBack.isBusy()){
            // wait for motor to reach position
        }
        StopDriving();
    }
    public void DriveBackwardLeft(double power, int distance){DriveForwardRight(-power, -distance);}
    public void DriveForward(double power, int distance){
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeft.setTargetPosition(-distance);
        robot.motorBack.setTargetPosition(distance);
        SetPower(-power,power,-power,power);
        while(opModeIsActive()&& robot.motorLeft.isBusy() && robot.motorBack.isBusy()){
            // wait for motor to reach position
        }
        StopDriving();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveBackward(double power, int distance){DriveForward(-power, -distance);}
    public void DriveRight(double power, int distance){
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeft.setTargetPosition(distance);
        robot.motorBack.setTargetPosition(distance);
        SetPower(power,power,-power,-power);
        while(opModeIsActive()&& robot.motorLeft.isBusy() && robot.motorBack.isBusy()){
            // wait for motor to reach position
        }
        StopDriving();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveLeft(double power, int distance){DriveRight(-power, -distance);}

    public void DriveEncoders(double left, double right, double front, double back, int dLeft, int dBack){
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeft.setTargetPosition(dLeft);
        robot.motorBack.setTargetPosition(dBack);
        SetPower(left, back, front, right);
        while(opModeIsActive()&& robot.motorLeft.isBusy() && robot.motorBack.isBusy()){
            // wait for motor to reach position
        }
        StopDriving();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}