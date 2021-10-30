package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import org.opencv.core.Mat;

public interface OpenCvConsumer {
    void processFrame(Mat frame);
}
