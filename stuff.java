package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Locale;
/**
 * Created by khadija on 9/1/2017.
 */
@Autonomous(name="stuff",group ="stuff")
public class stuff extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    DcMotor MotorLeft1 = null;
    DcMotor MotorLeft2 = null;
    DcMotor MotorRight1 = null;
    DcMotor MotorRight2 = null;


    @Override public void runOpMode() throws InterruptedException {

        MotorLeft1 = hardwareMap.dcMotor.get("MotorLeft1");
        MotorRight1 = hardwareMap.dcMotor.get("MotorRight1");
        MotorLeft2 = hardwareMap.dcMotor.get("MotorLeft2");
        MotorRight2 = hardwareMap.dcMotor.get("MotorRight2");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop, update the telementry, and set the power of the motors
        while (opModeIsActive()) {
            telemetry.update();
            MotorLeft1.setPower(-0.50);
            MotorRight1.setPower(-0.50);
            MotorLeft2.setPower(0.50);
            MotorRight2.setPower(0.50);
            sleep(2000);

            MotorLeft1.setPower(.75);
            MotorRight1.setPower(.75);
            MotorLeft2.setPower(-0.50);
            MotorRight2.setPower(-0.50);
            sleep(2000);

            MotorLeft1.setPower(0);
            MotorRight1.setPower(0);
            MotorLeft2.setPower(0);
            MotorRight2.setPower(0);


        }
    }


    void composeTelemetry(){



        telemetry.addAction(new Runnable() { @Override public void run()
        {
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
                .addData("calibration status", new Func<String>() {
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
                .addData("gravity", new Func<String>() {
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

        telemetry.addLine()
                .addData("LeftMotor1 Power = ", "%d", MotorLeft1.getPower());


        telemetry.addLine()
                .addData("LeftMotor2 Power = ","%d", MotorLeft2.getPower());

        telemetry.addLine()
                .addData("RightMotor1 Power = ","%d", MotorRight1.getPower());

        telemetry.addLine()
                .addData("RightMotor2 Power = ","%d", MotorRight2.getPower());

    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));

    }


}
