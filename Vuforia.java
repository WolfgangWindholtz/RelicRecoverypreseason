package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Robotics on 9/1/2017.
 */

public class Vuforia extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(0).setName("Tools");
        beacons.get(0).setName("Lego");
        beacons.get(0).setName("Gears");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix post = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
        }


    }
}
