package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by Robotics on 10/7/2017.
 */
    @Autonomous(name = "Balance2", group = "Balance2")
///@Disabled                            // Comment this out to add to the opmode list
    public class Balance2 extends LinearOpMode {
        Hardwaremap robot = new Hardwaremap();
        double x,y;
        @Override public void runOpMode() {
            robot.init(hardwareMap);
            composeTelemetry();
            waitForStart();
            robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            // Loop and update the dashboard
            while (opModeIsActive()) {
                telemetry.update();
                x = robot.angles.secondAngle/20;
                y = robot.angles.thirdAngle/20;
                while(Math.abs(x)>10 || Math.abs(y)>10){
                    x = robot.angles.secondAngle/10;
                    y = robot.angles.thirdAngle/10;
                    robot.motorRF.setPower(-y+x);
                    robot.motorLF.setPower(y+x);
                    robot.motorLB.setPower(y-x);
                    robot.motorRB.setPower(-y-x);
                    telemetry.update();
                }
                robot.motorRF.setPower(0);
                robot.motorLF.setPower(0);
                robot.motorLB.setPower(0);
                robot.motorRB.setPower(0);            }
        }

        //----------------------------------------------------------------------------------------------
        // Telemetry Configuration
        //----------------------------------------------------------------------------------------------

        void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the robot.imu that we will then display in separate lines.
            telemetry.addAction(new Runnable() { @Override public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity  = robot.imu.getGravity();
            }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override public String value() {
                            return robot.imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override public String value() {
                            return robot.imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return robot.gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(robot.gravity.xAccel*robot.gravity.xAccel
                                            + robot.gravity.yAccel*robot.gravity.yAccel
                                            + robot.gravity.zAccel*robot.gravity.zAccel));
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
    }

