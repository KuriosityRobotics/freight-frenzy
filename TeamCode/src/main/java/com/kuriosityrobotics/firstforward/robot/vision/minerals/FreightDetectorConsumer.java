package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import android.util.Log;
import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class FreightDetectorConsumer implements Runnable, OpenCvConsumer, Telemeter {
    private final PinholeCamera pinholeCamera;
    private ArrayList<Point> currentFreightPositions = new ArrayList<>();
    private final LocationProvider locationProvider;
    //private final VideoCapture capture;
    private volatile double lastFrameTime = -1;

    public FreightDetectorConsumer(LocationProvider locationProvider) {
        this.locationProvider = locationProvider;
        this.pinholeCamera = PinholeCamera.create();
        //this.capture = new VideoCapture(0);
    }

    public void processFrame(double cameraAngle, Mat img) {

        var startTime = System.currentTimeMillis();
        lastFrameTime = System.currentTimeMillis() - startTime;

        Vector3D robotVec = Vector3D.of(locationProvider.getPose().getX(), 0, locationProvider.getPose().getY());
        ArrayList<Point> newFreightPositions = new ArrayList<>();

        Log.v("freight", "start process");
        ArrayList<Vector2D> cubePixel = pinholeCamera.getCubePixelCoords(img);
        ArrayList<Vector2D> ballPixel = new ArrayList<>();//pinholeCamera.getBallPixelCoords(img);
        Log.v("freight", "end process");

        for (Vector2D cube : cubePixel){
            Point cubePos = pinholeCamera.findFreightPos(cube, cameraAngle, locationProvider.getPose().getHeading(), PinholeCamera.FreightType.CUBE);
            newFreightPositions.add(cubePos);
        }
        for (Vector2D ball : ballPixel){
            //Vector3D ballPos = pinholeCamera.getLocationOnField(robotVec, cameraAngle, locationProvider.getPose().getHeading(), ball, 1.375);
            //newFreightPositions.add(new Point(ballPos.getX(), ballPos.getZ()));
        }

        currentFreightPositions = newFreightPositions;
        Log.v("freight", newFreightPositions.toString());
        Log.v("freight", "processed frame");
    }

    public ArrayList<Point> getFreightPositions() {
        return currentFreightPositions;
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
        ArrayList<String> telemetry = new ArrayList<>();

        telemetry.add("FPS:  " + 1 / (lastFrameTime / 1000.));
        for (int i = 0; i < currentFreightPositions.size(); i++){
            telemetry.add(currentFreightPositions.get(i).toString());
        }

        return telemetry;
    }

    @Override
    public void run() {

    }
}
