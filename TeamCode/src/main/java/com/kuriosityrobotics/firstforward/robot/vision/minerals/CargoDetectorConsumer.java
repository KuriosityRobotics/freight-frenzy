package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import static java.lang.Math.PI;

import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.sensors.PoseProvider;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class CargoDetectorConsumer implements Runnable, OpenCvConsumer, Telemeter {
    static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    static final double FRAME_WIDTH = 1920;
    static final double FRAME_HEIGHT = 1080;
    static final double O_X = 995.588675691456;
    static final double O_Y = 599.3212928484164;
    static final double FOCAL_LENGTH = 1394.6027293299926;

    static final Vector3D CAMERA_POSITION = new Vector3D(8.075, 15.313, 0.185f);
    static final Rotation CAMERA_ROTATION = new Rotation(new Vector3D(-1, 0, 0), PI / 6, RotationConvention.VECTOR_OPERATOR);

    private final PinholeCamera pinholeCamera = new PinholeCamera(FOCAL_LENGTH, O_X, O_Y, FRAME_WIDTH, FRAME_HEIGHT, SENSOR_DIAGONAL, CAMERA_ROTATION, CAMERA_POSITION);
    private final ConcurrentHashMap<Point, Classifier.Recognition> detectedGameElements = new ConcurrentHashMap<>();
    private final PoseProvider poseProvider;
    private final AtomicReference<Pair<Mat, Pose>> latestFrame;
    private volatile double lastFrameTime = -1;

    public CargoDetectorConsumer(PoseProvider poseProvider) {
        latestFrame = new AtomicReference<>();

        this.poseProvider = poseProvider;
    }

    public void run() {
        while (!Thread.interrupted()) {
            var latest = latestFrame.getAndSet(null);
            if (latest != null) {
                var frame = latest.first;
                var pose = latest.second;

                runInferencingAndProjection(frame, pose);
                frame.release();
            }
        }
    }

    private void runInferencingAndProjection(Mat frame, Pose robotPose) {
        var startTime = System.currentTimeMillis();
        var detections = YoloV5Classifier.getInstance().findGameElementsOnMat(frame);
        lastFrameTime = System.currentTimeMillis() - startTime;

        var fieldToRobotRotation = new Rotation(new Vector3D(0, 1, 0), robotPose.heading, RotationConvention.VECTOR_OPERATOR);
        var fieldToRobotTranslate = new Vector3D(robotPose.x, 0, robotPose.y);

        detectedGameElements.clear();

        for (var detection : detections) {
            var u = detection.getLocation().centerX();
            var v = detection.getLocation().bottom;

            var fieldAbsolutePosition = fieldToRobotRotation
                    .applyInverseTo(pinholeCamera.unprojectFramePixelsToRay(u, v))
                    .add(fieldToRobotTranslate);

            detectedGameElements.put(new Point(fieldAbsolutePosition.getX(), fieldAbsolutePosition.getY()), detection);
        }

    }

    @Override
    public void processFrame(Mat frame) {
        var oldFrame = latestFrame.getAndSet(Pair.create(
                frame.clone(),
                poseProvider.getPose()));

        if (oldFrame != null)
            oldFrame.first.release();

        Imgproc.resize(frame, frame, new Size(416, 416));
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);
        for (var recognition : detectedGameElements.values()) {
            var rectF = recognition.getLocation();
            var rect = new Rect((int) rectF.left, (int) rectF.top, (int) rectF.width(), (int) rectF.height());
            Imgproc.rectangle(frame, rect, new Scalar(0, 0, 0));
            Imgproc.putText(frame, recognition.getDetectionType() + " " + recognition.getConfidence(), rect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255));
        }
    }

    public ConcurrentHashMap<Point, Classifier.Recognition> getDetectedGameElements() {
        return detectedGameElements;
    }

    @Override
    public String getName() {
        return "CargoDetectorConsumer";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("FPS:  " + 1 / (lastFrameTime / 1000.));
            detectedGameElements.entrySet().stream().map(Object::toString).forEach(this::add);
        }};
    }
}
