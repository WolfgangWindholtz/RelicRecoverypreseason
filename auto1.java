package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
/**
 * Created by khadija on 8/26/2017.
 */
@Autonomous(name="auto1",group="auto1")
public abstract class auto1 extends LinearOpMode {
        DcMotor motorLeft1 = null;
        DcMotor motorLeft2 = null;
        DcMotor motorRight1 = null;
        DcMotor motorRight2 = null;
@Override
    public void main() throws InterruptedException{
    motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
    motorRight1 = hardwareMap.dcMotor.get("motorRight1");
    motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
    motorRight2 = hardwareMap.dcMotor.get("motorRight2");


        waitForStart();

        while(opModeIsActive()){
            DcMotor.RunMode (int motor);


            telemetry.update();

            idle();
        }
    }
}
