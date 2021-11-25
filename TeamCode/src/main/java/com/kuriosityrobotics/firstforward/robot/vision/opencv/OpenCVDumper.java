package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import com.kuriosityrobotics.firstforward.robot.debug.FileDump;

import org.opencv.core.Mat;

public class OpenCVDumper implements OpenCvConsumer {
    private long lastCaptureTime;
    private boolean isOn;

    public OpenCVDumper(boolean debug) {
        this.isOn = debug;
    }

    @Override
    public void processFrame(Mat frame) {
        long millis = System.currentTimeMillis();
        if (isOn) {
            if (millis - lastCaptureTime > 500) {
                FileDump.dumpImage(frame);
                lastCaptureTime = millis;
            }
        }
    }

    public void toggleDumper() {
        isOn = !isOn;
    }
}