package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by sushruth on 10/20/17.
 */

public abstract class Processor extends LinearOpMode {
    Map bot = new Map();
    ElapsedTime runtime = new ElapsedTime();
    public final static double DEFAULT_POWER = .7;
    public final static int TICKSPERROTATION = 1120;
    static final double P_TURN_COEFF = .15;
    public final static int DIAMETEROFWHEEL = 4;
    static final double TURN_SPEED = 0.3;
    static final double DRIVE_SPEED = 0.6;
    static final double HEADING_THRESHOLD = 2;
    static final double OMNI_WHEEL_CIRCUMFERENCE = 4 * Math.PI;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));


    public void checkVu() {

        /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
        * it is perhaps unlikely that you will actually need to act on this pose information, but
        * we illustrate it nevertheless, for completeness. */
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) bot.relicTemplate.getListener()).getPose();
        telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            bot.tX = trans.get(0);
            bot.tY = trans.get(1);
            bot.tZ = trans.get(2);

            // X = vertical axis
            // Y = horizonatal Axis
            // Z = Depth Axis
            // Extract the rotational components of the target relative to the robot
            bot.rX = rot.firstAngle;
            bot.rY = rot.secondAngle;
            bot.rZ = rot.thirdAngle;
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        bot.vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
        telemetry.update();
    }

    public void checkCol() {
        checkVu();
        while (bot.columnToScore == null) {
            bot.vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
            if (bot.vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", bot.vuMark);

                bot.columnToScore = bot.vuMark;
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    // SEPARATE STATE LATER IS SCORE STATE
    // PASS IN PARAMETERS THAT WILL TELL HOW TO SCORE


    public void getOffStone() {
        //accesses the gyro values
        //drive based on Vuforia in
    }

    public void turn(double target) {
        Orientation ref = bot.imu.getAngularOrientation();

        double heading = ref.firstAngle;
        double correction;
        double error;

        double angleWanted = target + heading;

        ref = bot.imu.getAngularOrientation();
        double speed = turning(ref.firstAngle, angleWanted);
        while (speed != 0) {
            ref = bot.imu.getAngularOrientation();
            speed = turning(ref.firstAngle, angleWanted);
            accelerate(speed);
            recordTelemetry(target, angleWanted, ref, speed);
        }
        accelerate(0);
    }

    double turning(double firstAngle, double angleWanted) {
        double error;
        double correction;
        double speed;
        error = angleWanted - firstAngle;
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        correction = Range.clip(error * P_TURN_COEFF, -1, 1);

        telemetry.addData("correction", correction);


        if (Math.abs(error) <= HEADING_THRESHOLD) {
            return 0;
        } else {
            speed = TURN_SPEED * correction;
        }
        return speed;
    }

    public void recordTelemetry(double target, double angleWanted, Orientation ref, double speed) {
        telemetry.addData("first angle", ref.firstAngle);
        telemetry.addData("second angle", ref.secondAngle);
        telemetry.addData("third angle", ref.thirdAngle);
        telemetry.addData("target", target);
        telemetry.addData("speed ", speed);
        telemetry.addData("error", angleWanted - ref.firstAngle);
        telemetry.addData("angleWanted", angleWanted);
        telemetry.addData("motor power", bot.motorLF.getPower());


        telemetry.update();
    }

    private void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
        bot.motorLF.setPower(clip_speed);
        bot.motorRF.setPower(clip_speed);
        bot.motorRB.setPower(clip_speed);
        bot.motorLB.setPower(clip_speed);
    }

    public void go(double angle, double time) {

        bot.imu.getPosition();
        while (runtime.milliseconds() < time) {

            bot.motorLF.setPower(-DRIVE_SPEED);
            bot.motorRF.setPower(DRIVE_SPEED);
            bot.motorRB.setPower(DRIVE_SPEED);
            bot.motorLB.setPower(-DRIVE_SPEED);

        }
    }



    public void forward(double millisec) {

        runtime.reset();
        while (runtime.milliseconds() < millisec) {

            bot.motorLF.setPower(-DRIVE_SPEED);
            bot.motorRF.setPower(DRIVE_SPEED);
            bot.motorRB.setPower(DRIVE_SPEED);
            bot.motorLB.setPower(-DRIVE_SPEED);

        }
    }

    public void rampL(){
        double speed = Range.clip(.0001*runtime.milliseconds(),0,.2);
        bot.motorLF.setPower(speed);
        bot.motorRF.setPower(speed);
        bot.motorRB.setPower(-speed);
        bot.motorLB.setPower(-speed);
    }
    public void rampR(){
        double speed = Range.clip(.0001*runtime.milliseconds(),0,.2);
        bot.motorLF.setPower(-speed);
        bot.motorRF.setPower(-speed);
        bot.motorRB.setPower(speed);
        bot.motorLB.setPower(speed);
    }

    public void knockJewel(boolean isTeamRed) {
        bot.jewelServo.setPosition(.8);
        sleep(2000);
        int toTurn = checkJewel(isTeamRed, isSensorRed());
        telemetry.addData("blue", bot.colorSensor.blue());
        telemetry.addData("red", bot.colorSensor.red());
        turn(toTurn);
        sleep(500);
        bot.jewelServo.setPosition(0);
        sleep(500);

        turn(-toTurn);
        sleep(500);
    }

    public int checkJewel(boolean isTeamRed, boolean isSensorRed) {

        if (isTeamRed) {
            if (isSensorRed) {
                return 15;
            } else/*isTeamRed != isSensorRed*/ {
                return -15;
            }
        } else {
            if (isSensorRed) {
                return -15;
            } else/*isTeamRed != isSensorRed*/ {
                return 15;
            }
        }
    }

    public boolean isSensorRed() {

        telemetry.addData("blue", bot.colorSensor.blue());
        telemetry.addData("red", bot.colorSensor.red());
        return bot.colorSensor.red() > bot.colorSensor.blue();

    }

    public void gotoColumnLeft() {
        // the direction approaching the cryptobox changes depending on the side
        enterEnc();

        goPulsesRed(getColumnLeft());
        goAngle(2.5,0);

        stopBotMotors();
    }
    public void gotoColumnRight()
    {
        enterEnc();

        goPulses(getColumnRight());
        goAngle(0.25,180);

        stopBotMotors();
    }

    public void gotoColumnLeftEnc() {
        // the direction approaching the cryptobox changes depending on the side
        enterEnc();

        goColums(getColumnLeft());

        stopBotMotors();
    }

    public double getColumnDistance(int column) {
        double distance = 0;

        if (column == 3) {
            distance = 68;
        } else if (column == 2) {
            distance = 60.5;
        } else {
            distance = 53;
        }
        return distance;
    }

    public void goPulses(int numOfCol) {

        int count = 0;
        bot.colorServo.setPosition(.5);
        sleep(1000);
        runtime.reset();
        while (count < numOfCol&&runtime.milliseconds()<3500) {
            rampR();
            if (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                count++;

                if (numOfCol >= count) {
                    while (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                        bot.motorLF.setPower(-.2);
                        bot.motorRF.setPower(-.2);
                        bot.motorRB.setPower(.2);
                        bot.motorLB.setPower(.2);
                    }
                }
                // clear the column so the same column is not counted three time
            }
            telemetry.addData("distanceleft",getDistanceLeft());
            telemetry.addData("count", count);
            telemetry.update();
        }
        //sleep(700);

        //bot.colorServo.setPosition(0);\
        stopBotMotors();
    }
    public void goPulsesRed(int numOfCol) {
        int count = 0;
        bot.colorServo.setPosition(.5);
        sleep(1000);
        runtime.reset();
        while (count < numOfCol&&runtime.milliseconds()<3500) {

            rampL();

            if (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                count++;

                if (numOfCol >= count) {
                    while (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                        bot.motorLF.setPower(.2);
                        bot.motorRF.setPower(.2);
                        bot.motorRB.setPower(-.2);
                        bot.motorLB.setPower(-.2);
                    }
                }
                // clear the column so the same column is not counted three time
            }
            telemetry.addData("distanceleft",getDistanceLeft());
            telemetry.addData("count", count);
            telemetry.update();
        }
        //sleep(700);

        //bot.colorServo.setPosition(0);\
        stopBotMotors();
    }

    public double getDistanceLeft()
    {
        return  (bot.ultrasonicLeft.getVoltage()/3.3*1024);
    }

    public void getTelemetry()
    {
        telemetry.addData("UltrasonicDistanceLeft(in)", getDistanceLeft());
        telemetry.addData("Voltage Left ", getVoltagezLeft());
        telemetry.update();
    }
    public double getVoltagezLeft()
    {
        return bot.ultrasonicLeft.getVoltage();
    }


    public void drivingRangeForwardRed() {
        while(bot.rangeSensor.getDistance(DistanceUnit.INCH)>13.11)
        {
            bot.motorLF.setPower(-0.1);
            bot.motorRF.setPower((0.1));
            bot.motorLB.setPower(-0.1);
            bot.motorRB.setPower(0.1);
            getTelemetry();
        }

        stopBotMotors();

    }
    public void drivingRangeForwardBlue()
    {
        while(getDistanceLeft()>14.5)
        {
            bot.motorLF.setPower(-0.1);
            bot.motorRF.setPower((0.1));
            bot.motorLB.setPower(-0.1);
            bot.motorRB.setPower(0.1);
            getTelemetry();
        }

        stopBotMotors();
    }
    public void drivingRangeBackBlue()
    {
        while(getDistanceLeft()<13.38)
        {
            bot.motorRB.setPower(-0.1);
            bot.motorLB.setPower(0.1);
            bot.motorRF.setPower(-0.1);
            bot.motorLF.setPower(0.1);
            getTelemetry();
        }
        stopBotMotors();
    }
    public void drivingRangeBackRed()
    {
        while(bot.rangeSensor.getDistance(DistanceUnit.INCH)<13.38)
        {
            bot.motorRB.setPower(-0.1);
            bot.motorLB.setPower(0.1);
            bot.motorRF.setPower(-0.1);
            bot.motorLF.setPower(0.1);
            getTelemetry();

        }
        stopBotMotors();
    }
    public void align(double offset)
    {
        double error = angularOffset();
        double diff = offset -error;
        turn(diff);
    }
    public void raiseColorServo()
    {
        bot.colorServo.setPosition(0.5);
        sleep(1000);
    }
    public float angularOffset(){
        Orientation angleZZ = bot.imu.getAngularOrientation();
        float x = angleZZ.firstAngle;
        return x;
    }

    public void goColums(int count) {
        int c = 0;
        while (count > c) {

            goAngle(5, 180);
            stopBotMotors();


            telemetry.addData("count", count);
            telemetry.update();
            c++;
        }
        stopBotMotors();
    }

    public void goColumPrep(int count) {
        int c = 0;
        while (count > c) {

            goAngle(5, 0);
            stopBotMotors();


            telemetry.addData("count", count);
            telemetry.update();
            c++;
        }
        stopBotMotors();
    }


    public void score(int x) {
        bot.glyphServo4.setPosition(.4);
        bot.glyphServo3.setPosition(.6);
        runtime.reset();
        while (runtime.milliseconds() < 700) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        stopBotMotors();
        align(x);
        bot.glyphServo1.setPosition(0.4);
        bot.glyphServo2.setPosition(0.6);
        sleep(500);

        goAnglePower(3.3, 90, .3);
        sleep(500);
        //turn(30);





        goAnglePower(9, -90, .6);
        sleep(1000);

    }

    public void grabGlyph() {




        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.3);
        bot.glyphServo3.setPosition(.12);
        bot.glyphServo4.setPosition(1);
        sleep(1200);


        runtime.reset();

        //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
        while(runtime.milliseconds()<700) {
            bot.slideMotor.setPower(.8);
        }
        bot.slideMotor.setPower(0);
        sleep(700);
    }


    public void goAngle(double dist, double angle) {
        resetEnc();
        enterPosenc();
        double angel = Math.PI * angle / 180;
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = 1120 * distance;
        int ticksRF = (int) Math.round(ticks * Math.signum(y - x));
        int ticksLF = (int) Math.round(ticks * Math.signum(-y - x));
        int ticksLB = (int) Math.round(ticks * Math.signum(-y + x));
        int ticksRB = (int) Math.round(ticks * Math.signum(y + x));
        bot.motorLF.setTargetPosition(ticksLF);
        bot.motorRF.setTargetPosition(ticksRF);
        bot.motorRB.setTargetPosition(ticksRB);
        bot.motorLB.setTargetPosition(ticksLB);
        bot.motorRF.setPower(.5 * (y - x));
        bot.motorLF.setPower(.5 * (-y - x));
        bot.motorLB.setPower(.5 * (-y + x));
        bot.motorRB.setPower(.5 * (y + x));
        while (
                (bot.motorLB.isBusy() && bot.motorRB.isBusy() && bot.motorRF.isBusy() && bot.motorLF.isBusy())) {

            // Display it for the driver.

            telemetry.addData("Path2", "Running at %7d :%7d",
                    bot.motorLB.getCurrentPosition(),
                    bot.motorLF.getCurrentPosition(),
                    bot.motorRB.getCurrentPosition(),
                    bot.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at %7d :%7d",
                    bot.motorLB.getTargetPosition(),
                    bot.motorLF.getTargetPosition(),
                    bot.motorRB.getTargetPosition(),
                    bot.motorRF.getTargetPosition());
            telemetry.update();
        }

        stopBotMotors();

        sleep(250);
        enterEnc();
    }

   public void goAngleColumns(double dist, double angle,double power, double numOfCol){
       resetEnc();
       enterPosenc();
       double angel = Math.PI * angle / 180;
       double x = Math.cos(angel);
       double y = Math.sin(angel);
       double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
       double ticks = 1120 * distance;
       int ticksRF = (int) Math.round(ticks * Math.signum(y - x));
       int ticksLF = (int) Math.round(ticks * Math.signum(-y - x));
       int ticksLB = (int) Math.round(ticks * Math.signum(-y + x));
       int ticksRB = (int) Math.round(ticks * Math.signum(y + x));
       bot.motorLF.setTargetPosition(ticksLF);
       bot.motorRF.setTargetPosition(ticksRF);
       bot.motorRB.setTargetPosition(ticksRB);
       bot.motorLB.setTargetPosition(ticksLB);
       bot.motorRF.setPower(power * (y - x));
       bot.motorLF.setPower(power * (-y - x));
       bot.motorLB.setPower(power * (-y + x));
       bot.motorRB.setPower(power * (y + x));
       int count = 0;
       bot.colorServo.setPosition(.5);
       sleep(1000);
       runtime.reset();
       bot.motorRF.setPower(power * (y - x));
       bot.motorLF.setPower(power * (-y - x));
       bot.motorLB.setPower(power * (-y + x));
       bot.motorRB.setPower(power * (y + x));
       while (count < numOfCol) {
           while ((bot.motorLB.isBusy() && bot.motorRB.isBusy() && bot.motorRF.isBusy() && bot.motorLF.isBusy())) {

               telemetry.addData("Path2", "Running at %7d :%7d",
                       bot.motorLB.getCurrentPosition(),
                       bot.motorLF.getCurrentPosition(),
                       bot.motorRB.getCurrentPosition(),
                       bot.motorRF.getCurrentPosition());
               telemetry.addData("target", "Running at %7d :%7d",
                       bot.motorLB.getTargetPosition(),
                       bot.motorLF.getTargetPosition(),
                       bot.motorRB.getTargetPosition(),
                       bot.motorRF.getTargetPosition());
               telemetry.update();
               if (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                   count++;
                   if (numOfCol > count) {
                       while (bot.colorSensor2.getDistance(DistanceUnit.CM) < 25) {
                           bot.motorRF.setPower(power * (y - x));
                           bot.motorLF.setPower(power * (-y - x));
                           bot.motorLB.setPower(power * (-y + x));
                           bot.motorRB.setPower(power * (y + x));
                       }
                   }
                   if(count>=numOfCol){
                       break;
                   }
                   // clear the column so the same column is not counted three time
               }
               telemetry.addData("distanceleft",getDistanceLeft());
               telemetry.addData("count", count);
               telemetry.update();
           }
           stopBotMotors();
       }


       stopBotMotors();

       sleep(250);
       enterEnc();
    }


    public void resetEnc() {
        bot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void enterEnc() {
        bot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void enterPosenc() {
        bot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopBotMotors() {
        bot.motorRF.setPower(0);
        bot.motorLF.setPower(0);
        bot.motorLB.setPower(0);
        bot.motorRB.setPower(0);

    }


    public void goAnglePower(double dist, double angle, double power) {
        resetEnc();
        enterPosenc();
        double angel = Math.PI * angle / 180;
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = 1120 * distance;
        int ticksRF = (int) Math.round(ticks * Math.signum(y - x));
        int ticksLF = (int) Math.round(ticks * Math.signum(-y - x));
        int ticksLB = (int) Math.round(ticks * Math.signum(-y + x));
        int ticksRB = (int) Math.round(ticks * Math.signum(y + x));
        bot.motorLF.setTargetPosition(ticksLF);
        bot.motorRF.setTargetPosition(ticksRF);
        bot.motorRB.setTargetPosition(ticksRB);
        bot.motorLB.setTargetPosition(ticksLB);
        bot.motorRF.setPower(power * (y - x));
        bot.motorLF.setPower(power * (-y - x));
        bot.motorLB.setPower(power * (-y + x));
        bot.motorRB.setPower(power * (y + x));
        while (
                (bot.motorLB.isBusy() && bot.motorRB.isBusy() && bot.motorRF.isBusy() && bot.motorLF.isBusy())) {

            // Display it for the driver.

            telemetry.addData("Path2", "Running at %7d :%7d",
                    bot.motorLB.getCurrentPosition(),
                    bot.motorLF.getCurrentPosition(),
                    bot.motorRB.getCurrentPosition(),
                    bot.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at %7d :%7d",
                    bot.motorLB.getTargetPosition(),
                    bot.motorLF.getTargetPosition(),
                    bot.motorRB.getTargetPosition(),
                    bot.motorRF.getTargetPosition());
            telemetry.update();
        }

        stopBotMotors();

        sleep(250);
        enterEnc();
    }

    public void turnHeading(double target) {
        Orientation ref = bot.imu.getAngularOrientation();

        double angleWanted = target;

        ref = bot.imu.getAngularOrientation();
        double speed = turning(ref.firstAngle, angleWanted);
        while (speed != 0) {
            ref = bot.imu.getAngularOrientation();
            speed = turning(ref.firstAngle, angleWanted);
            accelerate(speed);
            recordTelemetry(target, angleWanted, ref, speed);
        }
        accelerate(0);
    }

    public int getColumnLeft() {
        int x = 0;
        if (bot.columnToScore == RelicRecoveryVuMark.RIGHT) {
            x = 1;
        }
        if (bot.columnToScore == RelicRecoveryVuMark.CENTER) {
            x = 2;
        }
        if (bot.columnToScore == RelicRecoveryVuMark.LEFT) {
            x = 3;
        }
        if(bot.columnToScore == RelicRecoveryVuMark.UNKNOWN){
            x = 1;
        }
        return x;
    }

    public int getColumnRight() {
        int x = 0;
        if (bot.columnToScore == RelicRecoveryVuMark.LEFT) {
            x = 1;
        }
        if (bot.columnToScore == RelicRecoveryVuMark.CENTER) {
            x = 2;
        }
        if (bot.columnToScore == RelicRecoveryVuMark.RIGHT) {
            x = 3;
        }
        if(bot.columnToScore == RelicRecoveryVuMark.UNKNOWN){
            x = 1;
        }
        return x;
    }

    public double getDistanceColumn(int column) {
        double ret = 0;
        if (column == 1) {
            ret = 45;
        } else if (column == 2) {
            ret = 52.5;
        } else if (column == 3) {
            ret = 60;
        } else {
            ret = 45;
        }
        return ret;
    }
    public float firstAngle(){
        Orientation angleAll = bot.imu.getAngularOrientation();
        float x = angleAll.firstAngle;
        return x;
    }


}