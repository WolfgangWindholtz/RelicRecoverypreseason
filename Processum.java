package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Rorobotics on 10/14/2017.
 */

public abstract class Processum extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();
    ElapsedTime runtime = new ElapsedTime();
    public final static double DEFAULT_POWER = .7;
    public final static int TICKSPERROTATION = 1120;
    static final double P_TURN_COEFF = .2;
    public final static int DIAMETEROFWHEEL = 4;
    static final double TURN_SPEED = 0.4;
    static final double DRIVE_SPEED = 0.6;
    static final double HEADING_THRESHOLD = 2;
    static final double OMNI_WHEEL_CIRCUMFERENCE = 2 * Math.PI;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));



    public void iteFB(float dist, double power) { //How far to travel in inches
        cessa();
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        float distance = dist / ((float) OMNI_WHEEL_CIRCUMFERENCE);
        int ticks = Math.round(1120 * distance);
        robot.motorLF.setTargetPosition(-ticks);//DISTANCE DETERMINES DIRECTION NOT POWER
        robot.motorRF.setTargetPosition(ticks);
        robot.motorRB.setTargetPosition(ticks);
        robot.motorLB.setTargetPosition(-ticks);
        robot.motorRF.setPower(power);
        robot.motorLF.setPower(power);
        robot.motorLB.setPower(power);
        robot.motorRB.setPower(power);
        sleep(250);
    }
    public void iteRL(float dist, double power) { ///How far to travel in inches
        cessa();
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        float distance = dist / ((float) OMNI_WHEEL_CIRCUMFERENCE);
        int ticks = Math.round(1120 * distance);
        robot.motorLF.setTargetPosition(-ticks);//DISTANCE DETERMINES DIRECTION NOT POWER
        robot.motorRF.setTargetPosition(-ticks);
        robot.motorRB.setTargetPosition(ticks);
        robot.motorLB.setTargetPosition(ticks);
        robot.motorRF.setPower(power);
        robot.motorLF.setPower(power);
        robot.motorLB.setPower(power);
        robot.motorRB.setPower(power);
        sleep(250);
    }

    public void iteAngle(float dist, double angle) {
        cessa();
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double angel = Math.PI*angle/180;
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = 1120 * distance;
        int ticksRF = (int)Math.round(ticks*.7*(-y+x));
        int ticksLF = (int)Math.round(ticks*.7*(y-x));
        int ticksLB = (int)Math.round(ticks*.7*(y+x));
        int ticksRB = (int)Math.round(ticks*.7*(y-x));
        robot.motorLF.setTargetPosition(ticksLF);
        robot.motorRF.setTargetPosition(ticksRF);
        robot.motorRB.setTargetPosition(ticksRB);
        robot.motorLB.setTargetPosition(ticksLB);
        robot.motorRF.setPower(.7 * (-y + x));
        robot.motorLF.setPower(.7 * (y + x));
        robot.motorLB.setPower(.7 * (y + x));
        robot.motorRB.setPower(.7 * (y - x));
        sleep(250);
    }
    public void cessa(){
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void verte(double target) {
        Orientation ref = robot.imu.getAngularOrientation();

        double heading = ref.firstAngle;
        double correction;
        double error;

        double angleWanted = target + heading;

        ref = robot.imu.getAngularOrientation();
        double speed = versa(ref.firstAngle, angleWanted);
        while(speed != 0 ){
            ref = robot.imu.getAngularOrientation();
            speed = versa(ref.firstAngle, angleWanted);
            accede(speed);
        }
        accede(0);
    }

    double versa(double firstAngle, double angleWanted) {
        double error;
        double correction;
        double speed;
        error = angleWanted - firstAngle;

        correction = Range.clip( error * P_TURN_COEFF,-1,1);

        if(Math.abs(error) <= HEADING_THRESHOLD){
            return 0;
        }
        else{
            speed = TURN_SPEED * correction;
        }
        return speed;
    }
    private void accede(double speed) {
        double clip_speed = Range.clip(-speed, -1, 1);
        robot.motorLF.setPower(clip_speed);
        robot.motorRF.setPower(clip_speed);
        robot.motorRB.setPower(clip_speed);
        robot.motorLB.setPower(clip_speed);
    }
    public void nissan_shitbox(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.motorLF.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.motorRF.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.motorRB.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.motorLB.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.motorLF.setTargetPosition(newLeftFrontTarget);
            robot.motorRF.setTargetPosition(newRightFrontTarget);
            robot.motorRB.setTargetPosition(newRightBackTarget);
            robot.motorLB.setTargetPosition(newLeftBackTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLF.setPower(Math.abs(speed));
            robot.motorRF.setPower(Math.abs(speed));
            robot.motorLB.setPower(Math.abs(speed));
            robot.motorRB.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLB.isBusy() && robot.motorRB.isBusy()&&robot.motorRF.isBusy()&&robot.motorLF.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newLeftFrontTarget,newRightBackTarget,newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLB.getCurrentPosition(),
                        robot.motorLF.getCurrentPosition(),
                        robot.motorRB.getCurrentPosition(),
                        robot.motorRF.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLB.setPower(0);
            robot.motorLF.setPower(0);
            robot.motorRB.setPower(0);
            robot.motorRF.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }


}
