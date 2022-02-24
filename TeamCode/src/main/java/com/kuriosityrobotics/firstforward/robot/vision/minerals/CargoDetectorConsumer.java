package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.sensors.PoseProvider;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class CargoDetectorConsumer implements Runnable, OpenCvConsumer, Telemeter {
    static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    static final double FRAME_WIDTH = 1920;
    static final double FRAME_HEIGHT = 1080;
    static final double O_X = 1132.95174  ;
    static final double O_Y = 692.88525;
    static final double FOCAL_LENGTH_X = 1803.64953;
    static final double FOCAL_LENGTH_Y = 1689.62755;

    private final PinholeCamera pinholeCamera;
    private final PhysicalCamera physicalCamera;
    private final ConcurrentHashMap<Point, Classifier.Recognition> detectedGameElements = new ConcurrentHashMap<>();
    private final PoseProvider poseProvider;
    private final AtomicReference<Pair<Mat, Pose>> latestFrame;
    private volatile double lastFrameTime = -1;

    public CargoDetectorConsumer(PoseProvider poseProvider, PhysicalCamera physicalCamera) {
        latestFrame = new AtomicReference<>();

        this.physicalCamera = physicalCamera;
        this.poseProvider = poseProvider;
        this.pinholeCamera = new PinholeCamera(
                FOCAL_LENGTH_X,
                FOCAL_LENGTH_Y,
                O_X,
                O_Y,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                SENSOR_DIAGONAL,
                physicalCamera.robotToCameraRotation().applyTo(poseProvider.getRotation()),
                physicalCamera.robotToCameraTranslation().add(poseProvider.getTranslation())
        );
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

        detectedGameElements.clear();

        for (var detection : detections) {
            var u = (double)detection.getLocation().centerX();
            var v =  (double)detection.getLocation().bottom;

            u = (u / 416) * FRAME_WIDTH;
            v = (v / 416) * FRAME_HEIGHT;

            var fieldAbsolutePosition = pinholeCamera.unprojectFramePixelsToRay(u, v);

            detectedGameElements.put(new Point(fieldAbsolutePosition.getX(), fieldAbsolutePosition.getZ()), detection);
        }

    }

    @Override
    public void processFrame(Mat frame) {
        var oldFrame = latestFrame.getAndSet(Pair.create(
                frame.clone(),
                poseProvider.getPose()));

        if (oldFrame != null)
            oldFrame.first.release();

        /*Imgproc.resize(frame, frame, new Size(416, 416));
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);
        for (var recognition : detectedGameElements.values()) {
            var rectF = recognition.getLocation();
            var rect = new Rect((int) rectF.left, (int) rectF.top, (int) rectF.width(), (int) rectF.height());
            Imgproc.rectangle(frame, rect, new Scalar(0, 0, 0));
            Imgproc.putText(frame, recognition.getDetectionType() + " " + recognition.getConfidence(), rect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255));
        }*/
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
