package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class CargoDetectorConsumer implements OpenCvConsumer, Telemeter {
    static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    static final double FRAME_WIDTH = 1920;
    static final double FRAME_HEIGHT = 1080;
    static final double O_X = 995.588675691456;
    static final double O_Y = 599.3212928484164;
    static final double FOCAL_LENGTH = 1394.6027293299926;

    static final Vector3D CAMERA_POSITION = new Vector3D(8.075, 15.313, 0.185f);
    static final Rotation CAMERA_ROTATION = new Rotation(new Vector3D(-1, 0, 0), PI / 6, RotationConvention.VECTOR_OPERATOR);

    private final PinholeCamera pinholeCamera = new PinholeCamera(FOCAL_LENGTH, O_X, O_Y, FRAME_WIDTH, FRAME_HEIGHT, SENSOR_DIAGONAL, CAMERA_ROTATION, CAMERA_POSITION);
    private final HashMap<Point, CargoDetector.GameElement> detectedGameElements = new HashMap<>();

    private final SensorThread sensorThread;

    public CargoDetectorConsumer(SensorThread sensorThread) {
        this.sensorThread = sensorThread;
    }

    @Override
    public void processFrame(Mat frame) {
        var detections = CargoDetector.getInstance().findGameElementsOnMat(frame);

        var robotPose = sensorThread.getPose();
        var fieldToRobotRotation = new Rotation(new Vector3D(0, 1, 0), robotPose.heading, RotationConvention.VECTOR_OPERATOR);
        var fieldToRobotTranslate = new Vector3D(robotPose.x, 0, robotPose.y);

        synchronized (detectedGameElements) {
            detectedGameElements.clear();

            for (var detection : detections) {
                var u = detection.cx();
                var v = detection.cy() + (detection.h() / 2); // go to bottom of bounding box bc we want the bit that's touching the field

                var fieldAbsolutePosition = fieldToRobotRotation
                        .applyInverseTo(pinholeCamera.unprojectFramePixelsToRay(u, v))
                        .add(fieldToRobotTranslate);

                detectedGameElements.put(new Point(fieldAbsolutePosition.getX(), fieldAbsolutePosition.getY()), detection.type());
            }
        }
    }

    public HashMap<Point, CargoDetector.GameElement> getDetectedGameElements() {
        synchronized (detectedGameElements) {
            return (HashMap<Point, CargoDetector.GameElement>) detectedGameElements.clone();
        }
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
        return detectedGameElements.entrySet().stream().map(Object::toString).collect(Collectors.toList());
    }
}
