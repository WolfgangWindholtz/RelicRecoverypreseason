package org.firstinspires.ftc.teamcode;


import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.*;
/**
 * Created by khadija on 11/3/2017.
 */

public class gltest {
    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    Mat mask = new Mat();
    Mat morphOutput = new Mat();

// remove some noise
Imgproc.blur(frame, blurredImage, new Size(7, 7));

// convert the frame to HSV
Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
}
