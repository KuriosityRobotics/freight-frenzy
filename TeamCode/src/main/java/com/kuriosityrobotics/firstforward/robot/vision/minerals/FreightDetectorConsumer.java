package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;

public class FreightDetectorConsumer implements Runnable, OpenCvConsumer, Telemeter {
    static{ System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    private final PinholeCamera pinholeCamera;
    private ArrayList<Point> currentFreightPositions = new ArrayList<>();
    private final LocationProvider locationProvider;
    private final VideoCapture capture;
    private volatile double lastFrameTime = -1;

    public FreightDetectorConsumer(LocationProvider locationProvider) {
        this.locationProvider = locationProvider;
        this.pinholeCamera = PinholeCamera.create();
        this.capture = new VideoCapture(0);
    }

    private volatile double cameraAngle = Math.toRadians(0);

    public void run() {
        while (!Thread.interrupted()) {
            Mat img = new Mat();
            capture.read(img);

            processFrame(cameraAngle, img);
        }
    }


    public void processFrame(double cameraAngle, Mat img) {
        var startTime = System.currentTimeMillis();
        lastFrameTime = System.currentTimeMillis() - startTime;

        currentFreightPositions.clear();
        ArrayList<Point> cubePixel = pinholeCamera.getCubePixelCoords(img);
        ArrayList<Point> ballPixel = pinholeCamera.getBallPixelCoords(img);

        for (Point cube : cubePixel){
            //currentFreightPositions.add(pinholeCamera.getLocationOnField(locationProvider.getPose()))
        }
        for (Point ball : ballPixel){
            //currentFreightPositions.add(pinholeCamera.getLocationOnField(locationProvider.getPose()))
        }
    }

    public ArrayList<Point> getFreightPositions() {
        return currentFreightPositions;
    }

    public void setCameraAngle(double cameraAngle){
        this.cameraAngle = cameraAngle;
    }

    @Override
    public String getName() {
        return "FreightDetectorConsumer";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("FPS:  " + 1 / (lastFrameTime / 1000.));
            add(currentFreightPositions.toString());
        }};
    }
}
