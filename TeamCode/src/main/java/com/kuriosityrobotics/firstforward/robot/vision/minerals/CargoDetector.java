package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import android.graphics.Bitmap;
import android.graphics.RectF;

import com.kuriosityrobotics.firstforward.robot.vision.minerals.detector.Classifier;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.detector.YoloV5Classifier;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Objects;

public class CargoDetector {
    /**
     * how close (in pixels) two points must be to be merged into one
     */
    public static final int CLOSENESS_THRESHOLD = 20;
    /**
     * location of the model
     */
    private static final String MODEL_FILE_NAME = "cargo.onnx";
    private static CargoDetector theCargoDetector;

    private final YoloV5Classifier net;
    private CargoDetector() {
        YoloV5Classifier net1;
        try {
            net1 = YoloV5Classifier.create(AppUtil.getInstance().getActivity().getAssets(), "best-int8.tflite", "labels.txt", true, 416);
        } catch (IOException e) {
            net1 = null;
            e.printStackTrace();
        }

        this.net = net1;
    }

    public static CargoDetector getInstance() {
        if (theCargoDetector == null)
            theCargoDetector = new CargoDetector();

        return theCargoDetector;
    }

    public synchronized ArrayList<Classifier.Recognition> findGameElementsOnMat(Mat frame) {
        Imgproc.resize(frame, frame, new Size(416, 416));
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);
        var bmp = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, bmp);

        var detections = net.recognizeImage(bmp);
        var filtered = new ArrayList<Classifier.Recognition>();

        outer: for (var detection : detections) {
            if (new RectF(0, 0, 416, 416).contains(detection.getLocation()) &&
                    detection.getConfidence() > .8) {
                for(var previous : filtered) {
                    if(Math.hypot(detection.getLocation().centerX() - previous.getLocation().centerX(), detection.getLocation().centerY() - previous.getLocation().centerY()) < CLOSENESS_THRESHOLD)
                        continue outer;
                }

                filtered.add(detection);
            }
        }

        return filtered;
    }

    enum GameElement {
        WAFFLE, BALL
    }

}
