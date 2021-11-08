package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import com.kuriosityrobotics.firstforward.robot.debug.FileDump;

import org.opencv.core.Mat;

public class OpenCVDumper implements OpenCvConsumer{
    private long lastCaptureTime;

    @Override
    public void processFrame(Mat frame) {
        long millis = System.currentTimeMillis();
        if (millis - lastCaptureTime > 500) {
            FileDump.dumpImage(frame);
            lastCaptureTime = millis;
        }
    }
}
