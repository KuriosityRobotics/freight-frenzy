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
    private Point bestFreightPosition;

    private final LocationProvider locationProvider;
    private final FreightDetectorHelper helper;
    private final boolean isBlue;
    private volatile double lastFrameTime = -1;

    public FreightDetectorConsumer(LocationProvider locationProvider, PinholeCamera pinholeCamera, boolean isBlue) {
        this.locationProvider = locationProvider;
        this.helper = new FreightDetectorHelper(pinholeCamera);
        this.currentFreightPositions = new ArrayList<>();
        this.isBlue = isBlue;
    }

    public void processFrame(double cameraAngle, Mat img) {
        List<Point> newFreightPositions = new ArrayList<>();

        //Log.v("freight", "start process");

        // balls are bad for our auto
        List<Point> cubePixel = helper.getCubePixels(img);
        List<Point> ballPixel = helper.getBallPixel(img);

        //Log.v("freight", "end process");

        for (Point cube : cubePixel){
            newFreightPositions.add(cube);
        }
        for (Point ball : ballPixel){
            newFreightPositions.add(ball);
        }

        currentFreightPositions = newFreightPositions;
        bestFreightPosition = helper.getBestFreight(currentFreightPositions, isBlue);

        //Log.v("freight", newFreightPositions.toString());
        //Log.v("freight", "processed frame");
        lastFrameTime = SystemClock.elapsedRealtime();
    }

    public List<Point> getFreightPositions() {
        return currentFreightPositions;
    }

    public Point getBestFreightPosition(){
        return bestFreightPosition;
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
