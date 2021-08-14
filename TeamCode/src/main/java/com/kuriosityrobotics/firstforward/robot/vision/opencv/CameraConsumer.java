package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import org.opencv.core.Mat;

public interface CameraConsumer {
    void  processFrame(Mat frame);
}
