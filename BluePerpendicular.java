package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Sushr on 12/16/2017.
 */
@Autonomous(name = "BluePerpendicular", group = "jfjf")
public class BluePerpendicular extends Processor {
    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        checkCol();
        grabGlyph();
        align(0);

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        goAnglePower(20,180,.4);
        align(0);
        turn(90);
        align(90);
        goAnglePower(1.3,0,.3);
        raiseColorServo();
        adjust(.045);
        sleep(200);
        adjust(.045);
        adjust(.045);
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

    /**
     * Created by Robotics on 12/15/2017.
     */

    public static class protoMap {
        public BNO055IMU imu;
        HardwareMap hwMap = null;
        DcMotor motorLF;
        DcMotor motorLB;
        DcMotor motorRF;
        DcMotor motorRB;
        DcMotor slideMotor;
        DcMotor intakeMotor;
        Servo dumpServo1;
        Servo dumpServo2;
        Servo jewelServo;


        ModernRoboticsI2cRangeSensor rangeSensor = null;

        ColorSensor colorSensor = null;

        TouchSensor touchSensor;

        int cameraMonitorViewId;
        VuforiaTrackables relicTrackables;
        VuforiaTrackable relicTemplate;

        RelicRecoveryVuMark columnToScore;

        double tX;
        double tY;
        double tZ;

        double rX;
        double rY;
        double rZ;
        RelicRecoveryVuMark vuMark;

        VuforiaLocalizer vuforia;

        IntegratingGyroscope gyro;
        NavxMicroNavigationSensor navxMicro;

        // A timer helps provide feedback while calibration is taking place
        ElapsedTime timer = new ElapsedTime();
        public void init(HardwareMap ahwMap) {

            // save reference to HW Map
            hwMap = ahwMap;

            motorLB = hwMap.dcMotor.get("motorLB");
            motorLF = hwMap.dcMotor.get("motorLF");
            motorRF = hwMap.dcMotor.get("motorRF");
            motorRB = hwMap.dcMotor.get("motorRB");

            slideMotor = hwMap.dcMotor.get("slideMotor");
            dumpServo1 = hwMap.servo.get("dumpServo1");
            dumpServo2 = hwMap.servo.get("dumpServo2");
            jewelServo = hwMap.servo.get("jewelServo");

            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
            colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
            touchSensor = hwMap.touchSensor.get("touchSensor");

            motorLF.setDirection(DcMotor.Direction.FORWARD);
            motorRB.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorLB.setDirection(DcMotor.Direction.FORWARD);
            slideMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            param.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";

            param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(param);

            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

            relicTrackables.activate();
            colorSensor.enableLed(true);

            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        }
    }
}