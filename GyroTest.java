package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

/**
 * Created by WHHS robotics on 9/23/17.
 */
@Autonomous(name = "GyroTest" , group ="Concept")

public class GyroTest extends LinearOpMode {
    public BNO055IMU imu;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    Orientation angles;
    Acceleration gravity;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        timer.reset();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        composeTelemetry();

        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
        //analyzes the Pictogram image
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            Orientation nav = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

            telemetry.addLine()
                    .addData("heading", formatAngle(nav.angleUnit, nav.firstAngle))
                    .addData("roll", formatAngle(nav.angleUnit, nav.secondAngle))
                    .addData("pitch", "%s deg", formatAngle(nav.angleUnit, nav.thirdAngle));
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Created by Robotics on 12/15/2017.
     */

    public abstract static class protoProcessor extends LinearOpMode {
        BluePerpendicular.protoMap bot = new BluePerpendicular.protoMap();
        ElapsedTime runtime = new ElapsedTime();
        public final static double DEFAULT_POWER = .7;
        public final static int TICKSPERROTATION = 1120;
        static final double P_TURN_COEFF = .07;
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
            while(bot.columnToScore == null) {
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
        public void turn(double target) {
            Orientation ref = bot.navxMicro.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            double heading = ref.firstAngle;
            double correction;
            double error;

            double angleWanted = target + heading;

            ref = bot.imu.getAngularOrientation();
            double speed = turning(ref.firstAngle, angleWanted);
            while(speed != 0 ){
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

            correction = Range.clip( error * P_TURN_COEFF,-1,1);

            telemetry.addData("correction",correction);


            if(Math.abs(error) <= HEADING_THRESHOLD){
                return 0;
            }
            else{
                speed = TURN_SPEED * correction;
            }
            return speed;
        }

        public void recordTelemetry(double target, double angleWanted, Orientation ref, double speed) {
            telemetry.addData("first angle",ref.firstAngle);
            telemetry.addData("second angle",ref.secondAngle);
            telemetry.addData("third angle",ref.thirdAngle);
            telemetry.addData("target",target);
            telemetry.addData("speed ",speed);
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
        public void knockJewel(boolean isTeamRed){
            bot.jewelServo.setPosition(.9);
            sleep(2000);
            int toTurn = checkJewel(isTeamRed,isSensorRed());
            telemetry.addData("blue", bot.colorSensor.blue());
            telemetry.addData("red", bot.colorSensor.red());
            turn(toTurn);
            sleep(500);
            bot.jewelServo.setPosition(.2);
            sleep(500);

            turn(-toTurn);
            sleep(500);
        }

        public  int checkJewel(boolean isTeamRed, boolean isSensorRed){

            if(isTeamRed){
                if(isSensorRed){
                    return 15;
                }
                else/*isTeamRed != isSensorRed*/{
                    return -15;
                }
            }
            else{
                if(isSensorRed){
                    return -15;
                }
                else/*isTeamRed != isSensorRed*/{
                    return 15;
                }
            }
        }

        public boolean isSensorRed(){

            telemetry.addData("blue", bot.colorSensor.blue());
            telemetry.addData("red", bot.colorSensor.red());
            return  bot.colorSensor.red() > bot.colorSensor.blue();

        }
        public void gotoColumnRightEnc() {
            enterEnc();

            if (bot.columnToScore == RelicRecoveryVuMark.LEFT) {
                goColumnR(1);
            }
            if (bot.columnToScore == RelicRecoveryVuMark.CENTER) {
                goColumnR(2);
            }
            if (bot.columnToScore == RelicRecoveryVuMark.RIGHT) {
                goColumnR(3);
            }

        }

        public void gotoColumnLeftEnc() {
            // the direction approaching the cryptobox changes depending on the side
            enterEnc();


            if (bot.columnToScore == RelicRecoveryVuMark.RIGHT) {
                goColumnL(1);
            }
            if (bot.columnToScore == RelicRecoveryVuMark.CENTER) {
                goColumnL(2);
            }
            if (bot.columnToScore == RelicRecoveryVuMark.LEFT) {
                goColumnL(3);
            }

            stopBotMotors();
        }
        public void goColumnL(int count){
            int c = 0;
            while(count > c){
                goAngle(4.6,180);
                stopBotMotors();


                telemetry.addData("count",count );
                telemetry.update();
                c++;
            }
            stopBotMotors();
        }


        public void goColumnR(int count){
            int c = 0;
            while(count > c){
                goAngle(4.6,0);
                stopBotMotors();


                telemetry.addData("count",count );
                telemetry.update();
                c++;
            }
            stopBotMotors();
        }

        public void score() {
            stopBotMotors();
            bot.dumpServo2.setPosition(.5);
            bot.dumpServo1.setPosition(.5);
            sleep(1000);
            bot.dumpServo2.setPosition(.03);
            bot.dumpServo1.setPosition(.9);
            sleep(1000);
            goAnglePower(7,90,.3);
            sleep(1000);
            //turn(30);

            goAnglePower(7,-90,.5);
            sleep(1000);

        }



        public void goAngle(double dist, double angle) {
            resetEnc();
            enterPosenc();
            double angel = Math.PI*angle/180;
            double x = Math.cos(angel);
            double y = Math.sin(angel);
            double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
            double ticks = 1120 * distance;
            int ticksRF = (int)Math.round(ticks*Math.signum(y-x));
            int ticksLF = (int)Math.round(ticks*Math.signum(-y-x));
            int ticksLB = (int)Math.round(ticks*Math.signum(-y+x));
            int ticksRB = (int)Math.round(ticks*Math.signum(y+x));
            bot.motorLF.setTargetPosition(ticksLF);
            bot.motorRF.setTargetPosition(ticksRF);
            bot.motorRB.setTargetPosition(ticksRB);
            bot.motorLB.setTargetPosition(ticksLB);
            bot.motorLF.setPower(.7*(-y-x));
            bot.motorRF.setPower(.7*(y-x));
            bot.motorRB.setPower(.7*(y+x));
            bot.motorLB.setPower(.7*(-y+x));
            while (
                    (bot.motorLB.isBusy() && bot.motorRB.isBusy()&&bot.motorRF.isBusy()&&bot.motorLF.isBusy())) {

                // Display it for the driver.

                telemetry.addData("Path2",  "Running at %7d :%7d",
                        bot.motorLB.getCurrentPosition(),
                        bot.motorLF.getCurrentPosition(),
                        bot.motorRB.getCurrentPosition(),
                        bot.motorRF.getCurrentPosition());
                telemetry.addData("target",  "Running at %7d :%7d",
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


        public void resetEnc(){
            bot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void enterEnc(){
            bot.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        public void enterPosenc(){
            bot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public void stopBotMotors(){
            bot.motorRF.setPower(0);
            bot.motorLF.setPower(0);
            bot.motorLB.setPower(0);
            bot.motorRB.setPower(0);

        }

        public void goAnglePower(double dist, double angle,double power) {
            resetEnc();
            enterPosenc();
            double angel = Math.PI*angle/180;
            double x = Math.cos(angel);
            double y = Math.sin(angel);
            double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
            double ticks = 1120 * distance;
            int ticksRF = (int)Math.round(ticks*Math.signum(y-x));
            int ticksLF = (int)Math.round(ticks*Math.signum(-y-x));
            int ticksLB = (int)Math.round(ticks*Math.signum(-y+x));
            int ticksRB = (int)Math.round(ticks*Math.signum(y+x));
            bot.motorLF.setTargetPosition(ticksLF);
            bot.motorRF.setTargetPosition(ticksRF);
            bot.motorRB.setTargetPosition(ticksRB);
            bot.motorLB.setTargetPosition(ticksLB);
            bot.motorLF.setPower(power*(-y-x));
            bot.motorRF.setPower(power*(y-x));
            bot.motorRB.setPower(power*(y+x));
            bot.motorLB.setPower(power*(-y+x));
            while (
                    (bot.motorLB.isBusy() && bot.motorRB.isBusy()&&bot.motorRF.isBusy()&&bot.motorLF.isBusy())) {

                // Display it for the driver.

                telemetry.addData("Path2",  "Running at %7d :%7d",
                        bot.motorLB.getCurrentPosition(),
                        bot.motorLF.getCurrentPosition(),
                        bot.motorRB.getCurrentPosition(),
                        bot.motorRF.getCurrentPosition());
                telemetry.addData("target",  "Running at %7d :%7d",
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
    }

    /**
     * Created by Robotics on 12/15/2017.
     */

    public static class protoTeleMap {
        public BNO055IMU imu;
        HardwareMap hwMap = null;
        DcMotor motorLF;
        DcMotor motorLB;
        DcMotor motorRF;
        DcMotor motorRB;
        DcMotor slideMotor;
        DcMotor relicMotor;
        DcMotor intakeMotor;

        Servo dumpServo1;
        Servo dumpServo2;
        Servo jewelServo;
        Servo relicFingers;
        Servo relicWrist;


        ModernRoboticsI2cRangeSensor rangeSensor = null;

        ColorSensor colorSensor = null;


        RelicRecoveryVuMark columnToScore;


        ElapsedTime runtime = new ElapsedTime();


        public void init(HardwareMap ahwMap) {

            // save reference to HW Map
            hwMap = ahwMap;
            imu = hwMap.get(BNO055IMU.class, "imu");
            motorLB = hwMap.dcMotor.get("motorLB");
            motorLF = hwMap.dcMotor.get("motorLF");
            motorRF = hwMap.dcMotor.get("motorRF");
            motorRB = hwMap.dcMotor.get("motorRB");
            relicMotor = hwMap.dcMotor.get("relicMotor");

            slideMotor = hwMap.dcMotor.get("slideMotor");
            dumpServo1 = hwMap.servo.get("dumpServo1");
            dumpServo2 = hwMap.servo.get("dumpServo2");
            jewelServo = hwMap.servo.get("jewelServo");
            relicFingers = hwMap.servo.get("relicFingers");
            relicWrist = hwMap.servo.get("relicWrist");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
            colorSensor = hwMap.get(ColorSensor.class,"colorSensor");

            motorLF.setDirection(DcMotor.Direction.FORWARD);
            motorRB.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorLB.setDirection(DcMotor.Direction.FORWARD);
            slideMotor.setDirection(DcMotor.Direction.FORWARD);

            motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            jewelServo.setPosition(.2);
            //glyphServo1.setPosition(0.4);
            //glyphServo2.setPosition(0.6);


        }

    }

    /**
     * Created by Robotics on 12/15/2017.
     */
    @TeleOp(group = "proto",name = "protoTele")
    public static class protoTeleOp extends OpMode {
        protoTeleMap bot = new protoTeleMap();
        double xpow;
        double ypow;
        double zpow;
        double x,y,z;
        double rightx;
        boolean toggle = false;

        @Override
        public void init() {
            bot.init(hardwareMap);
        }

        @Override
        public void loop() {
            //https://www.vexforum.com/index.php/7873-programming-mecanum-wheels/0
            zpow = gamepad1.right_stick_x;
            ypow = gamepad1.left_stick_y;
            xpow = gamepad1.left_stick_x;

            bot.motorLF.setPower(.7*(-ypow-xpow-toggle(toggle,zpow)));
            bot.motorRF.setPower(.7*(ypow-xpow-toggle(toggle,zpow)));
            bot.motorRB.setPower(.7*(ypow+xpow-toggle(toggle,zpow)));
            bot.motorLB.setPower(.7*(-ypow+xpow-toggle(toggle,zpow)));

            if(gamepad1.dpad_left){
                bot.motorLF.setPower(.7);
                bot.motorRF.setPower(.7);
                bot.motorRB.setPower(-.7);
                bot.motorLB.setPower(-.7);
            }
            if(gamepad1.dpad_right){
                bot.motorLF.setPower(-.7);
                bot.motorRF.setPower(-.7);
                bot.motorRB.setPower(.7);
                bot.motorLB.setPower(.7);
            }
            if(gamepad1.dpad_up){
                bot.motorLF.setPower(-.7);
                bot.motorRF.setPower(.7);
                bot.motorRB.setPower(.7);
                bot.motorLB.setPower(-.7);
            }
            if(gamepad1.dpad_down){
                bot.motorLF.setPower(.7);
                bot.motorRF.setPower(-.7);
                bot.motorRB.setPower(-.7);
                bot.motorLB.setPower(.7);
            }
            else{
                bot.motorLF.setPower(0);
                bot.motorRF.setPower(0);
                bot.motorRB.setPower(0);
                bot.motorLB.setPower(0);
            }

            double slidePower = -gamepad2.left_stick_y;

            if(slidePower>0)
            {
                slidePower /= 4;
            }

            bot.slideMotor.setPower(slidePower);

            if(gamepad1.a){
                if(!toggle){
                    toggle = true;
                }
                else {
                    toggle = false;
                }

            }


            double relicPower = gamepad2.right_stick_y;
            bot.relicMotor.setPower(relicPower);

            if(gamepad2.a)  // gripGlyphs
            {
                intakeOut();
            }
            if(gamepad2.x)  // openLeft
            {
                intakeIn();
            }
            if(gamepad2.b)  // openRight
            {
                intakeOff();
            }
            if(gamepad2.right_bumper){
                dump();
            }
            if(gamepad2.left_bumper){
                load();
            }
            if(gamepad2.dpad_left){
                fingersClose();  // fingers closed for relic
            }
            if(gamepad2.dpad_right){
                fingersOpen(); // opens finger servo for relic
            }
            if(gamepad2.dpad_up){
                wristUp();   // brings wrist up for relic
            }
            if(gamepad2.dpad_down){
                wristDown(); // bring wrist down for relic
            }

        }

        public void fingersOpen(){
            bot.relicFingers.setPosition(.6);
        }

        public void fingersClose(){
            bot.relicFingers.setPosition(.95);
        }

        public void wristUp() {
            bot.relicWrist.setPosition(.7);
        }

        public void wristDown() {
            bot.relicWrist.setPosition(0);
        }
        public void dump() {
            bot.dumpServo2.setPosition(.5);
            bot.dumpServo1.setPosition(.5);
        }
        public void load() {
            bot.dumpServo2.setPosition(.03);
            bot.dumpServo1.setPosition(.9);
        }
        public void intakeIn() {
            bot.intakeMotor.setPower(.5);
        }
        public void intakeOut() {
            bot.intakeMotor.setPower(-.5);
        }
        public void intakeOff() {
            bot.intakeMotor.setPower(0);
        }

        public double toggle(boolean toggle, double power){
            if(toggle){
                return power * .4;
            }
            else{
                return power;
            }

        }
    }
}
