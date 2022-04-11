package com.kuriosityrobotics.firstforward.robot.vision.minerals;


import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class FreightDetectorConsumer implements OpenCvConsumer, Telemeter {
    private List<Point> currentFreightPositions;
    private final LocationProvider locationProvider;
    private final FreightDetectorHelper helper;
    private volatile double lastFrameTime = -1;

    public FreightDetectorConsumer(LocationProvider locationProvider, PinholeCamera camera) {
        this.locationProvider = locationProvider;
        this.helper = new FreightDetectorHelper(camera);
        this.currentFreightPositions = new ArrayList<>();
    }

    public void processFrame(double cameraAngle, Mat img) {
        List<Point> newFreightPositions = new ArrayList<>();

        Log.v("freight", "start process");

        // balls are bad for our auto
        List<Vector2D> cubePixel = helper.getCargoPixels(img);
//        List<Vector2D> ballPixel = helper.getBallPixelCoords(img);

        Log.v("freight", "end process");

        for (Vector2D cube : cubePixel){
            Point cubePos = helper.findFreightPos(cube, cameraAngle, locationProvider.getPose(), FreightDetectorHelper.FreightType.CUBE);
            newFreightPositions.add(cubePos);
        }
//        for (Vector2D ball : ballPixel){
//            Point ballPos = helper.findFreightPos(ball, cameraAngle, locationProvider.getPose(), FreightDetectorHelper.FreightType.CUBE);
//            newFreightPositions.add(ballPos);
//        }

        currentFreightPositions = newFreightPositions;
        Log.v("freight", newFreightPositions.toString());
        Log.v("freight", "processed frame");
        lastFrameTime = SystemClock.elapsedRealtime();
    }

    public List<Point> getFreightPositions() {
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
        List<String> retVal = new ArrayList<>();

        retVal.add("FPS:  " + 1 / ((System.currentTimeMillis() - lastFrameTime) / 1000.));
        for (int i = 0; i < currentFreightPositions.size(); i++){
            retVal.add(currentFreightPositions.get(i).toString());
        }

        return retVal;
    }
}
