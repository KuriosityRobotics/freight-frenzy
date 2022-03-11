package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;

public class OpenCVDumper implements OpenCvConsumer {
    private long lastCaptureTime;

    @Override
    public void processFrame(double _cameraAngle, Mat frame) {
        if (!Robot.DEBUG) {
            return;
        }

        long millis = System.currentTimeMillis();
        if (millis - lastCaptureTime > 500) {
            File file = new File(AppUtil.ROBOT_DATA_DIR + "/" + "webcam-frame-" + new Date().getTime() + ".jpg");
            MatOfByte mob = new MatOfByte();
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2BGR);
            Imgcodecs.imencode(".jpg", frame, mob);
            try (FileOutputStream stream = new FileOutputStream(file)) {
                stream.write(mob.toArray());
            } catch (IOException e) {
                Log.w("VisionDump", e);
            }
            lastCaptureTime = millis;
        }
    }
}
