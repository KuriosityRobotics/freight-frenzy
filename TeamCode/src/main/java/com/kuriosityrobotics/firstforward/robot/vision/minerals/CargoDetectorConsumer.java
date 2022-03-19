package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class CargoDetectorConsumer implements Runnable, OpenCvConsumer, Telemeter {

    private final PinholeCamera pinholeCamera;
    private final ConcurrentHashMap<Point, Recognition> detectedGameElements = new ConcurrentHashMap<>();
    private final LocationProvider locationProvider;
    private final AtomicReference<Pair<Mat, Pose>> latestFrame;
    private volatile double lastFrameTime = -1;

    public CargoDetectorConsumer(LocationProvider locationProvider) {
        latestFrame = new AtomicReference<>();

        this.locationProvider = locationProvider;
        this.pinholeCamera = PinholeCamera.create();
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

            u = (u / 416) * PhysicalCamera.FRAME_WIDTH;
            v = (v / 416) * PhysicalCamera.FRAME_HEIGHT;

            var fieldAbsolutePosition = pinholeCamera.getLocationOnField(Vector3D.of(robotPose.getX(), 0, robotPose.getY()), robotPose.heading + cameraAngle, u, v);

            detectedGameElements.put(new Point(fieldAbsolutePosition.getX(), fieldAbsolutePosition.getZ()), detection);
        }

    }

    private volatile double cameraAngle = 0;

    @Override
    public void processFrame(double cameraAngle, Mat frame) {
        //1920x1080

        var oldFrame = latestFrame.getAndSet(Pair.create(
                frame.clone(),
                locationProvider.getPose()));

        if (oldFrame != null)
            oldFrame.first.release();
        this.cameraAngle = cameraAngle;

/*        var loc = pinholeCamera.getLocationOnFrame(new Vector3D(35, 0, 50.5));
        Imgproc.circle(frame, new org.opencv.core.Point(
                loc.getX(),
                loc.getY()
        ), 4, new Scalar(255, 255, 255));*/

        /*Imgproc.resize(frame, frame, new Size(416, 416));
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);
        for (var recognition : detectedGameElements.values()) {
            var rectF = recognition.getLocation();
            var rect = new Rect((int) rectF.left, (int) rectF.top, (int) rectF.width(), (int) rectF.height());
            Imgproc.rectangle(frame, rect, new Scalar(0, 0, 0));
            Imgproc.putText(frame, recognition.getDetectionType() + " " + recognition.getConfidence(), rect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255));
        }*/
    }

    public ConcurrentHashMap<Point, Recognition> getDetectedGameElements() {
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
