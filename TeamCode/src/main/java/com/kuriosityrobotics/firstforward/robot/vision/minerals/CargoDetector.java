package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;

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

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private final Net net;

    private CargoDetector() {
        this.net = Dnn.readNet(MODEL_FILE_NAME);
    }

    public static CargoDetector getInstance() {
        if (theCargoDetector == null)
            theCargoDetector = new CargoDetector();

        return theCargoDetector;
    }

    public synchronized ArrayList<Detection> findGameElementsOnMat(Mat mat) {
        net.setInput(Dnn.blobFromImage(mat, 1 / 255., new Size(416, 416), new Scalar(0), true));
        var detectorOutputLayer = net.forward("output");
        detectorOutputLayer = detectorOutputLayer.reshape(1, (int) detectorOutputLayer.total() / 7);

        var detections = new ArrayList<Detection>();

        detections:
        for (var i = 0; i < detectorOutputLayer.rows(); i++) {
            var confidence = detectorOutputLayer.get(i, 4)[0];

            if (confidence > .6) {
                var cx = detectorOutputLayer.get(i, 0)[0];
                var cy = detectorOutputLayer.get(i, 1)[0];
                var w = detectorOutputLayer.get(i, 2)[0];
                var h = detectorOutputLayer.get(i, 3)[0];
                var type = GameElement.values()[(int) Math.round(detectorOutputLayer.get(i, 5)[0])];

                for (var previousDetection : detections)
                    if (Math.hypot(cx - previousDetection.cx, cy - previousDetection.cy) < CLOSENESS_THRESHOLD)
                        continue detections;


                detections.add(new Detection(confidence, cx, cy, w, h, type));
            }
        }

        return detections;
    }

    enum GameElement {
        WAFFLE, BALL
    }

    static final class Detection {
        private final double confidence;
        private final double cx;
        private final double cy;
        private final double w;
        private final double h;
        private final GameElement type;

        Detection(double confidence, double cx, double cy, double w, double h, GameElement type) {
            this.confidence = confidence;
            this.cx = cx;
            this.cy = cy;
            this.w = w;
            this.h = h;
            this.type = type;
        }

        public Rect getRect() {
            return new Rect(new Point(cx - w / 2, cy - h / 2), new Point(cx + w / 2, cy + h / 2));
        }

        public double confidence() {
            return confidence;
        }

        public double cx() {
            return cx;
        }

        public double cy() {
            return cy;
        }

        public double w() {
            return w;
        }

        public double h() {
            return h;
        }

        public GameElement type() {
            return type;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this)
                return true;
            if (obj == null || obj.getClass() != this.getClass())
                return false;
            var that = (Detection) obj;
            return Double.doubleToLongBits(this.confidence) == Double.doubleToLongBits(that.confidence) &&
                    Double.doubleToLongBits(this.cx) == Double.doubleToLongBits(that.cx) &&
                    Double.doubleToLongBits(this.cy) == Double.doubleToLongBits(that.cy) &&
                    Double.doubleToLongBits(this.w) == Double.doubleToLongBits(that.w) &&
                    Double.doubleToLongBits(this.h) == Double.doubleToLongBits(that.h) &&
                    Objects.equals(this.type, that.type);
        }

        @Override
        public int hashCode() {
            return Objects.hash(confidence, cx, cy, w, h, type);
        }

        @Override
        public String toString() {
            return "Detection[" +
                    "confidence=" + confidence + ", " +
                    "cx=" + cx + ", " +
                    "cy=" + cy + ", " +
                    "w=" + w + ", " +
                    "h=" + h + ", " +
                    "type=" + type + ']';
        }

    }
}
