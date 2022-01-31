package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final TeamMarkerDetector teamMarkerDetector;
    private final OpenCVDumper openCVDumper;

    public SingleManagedCamera singleManagedCamera;
    private final Robot robot;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, VuforiaLocalizationConsumer vuforiaLocalizationConsumer) {
        this.robot = robot;

        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;
        this.openCVDumper = new OpenCVDumper(robot.isDebug());
        this.teamMarkerDetector = new TeamMarkerDetector();
        this.singleManagedCamera = new SingleManagedCamera(robot.camera, vuforiaLocalizationConsumer, openCVDumper, teamMarkerDetector);

        robot.telemetryDump.registerTelemeter(this);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
        telemetryData.addAll(vuforiaLocalizationConsumer.logPositionandDetection());
        telemetryData.add("Team marker location: " + teamMarkerDetector.getLocation());
        telemetryData.add("Update time: " + updateTime);
        return telemetryData;
    }

    @Override
    public String getName() {
        return "VisionThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public void run() {
        while (robot.running()) {
            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        this.vuforiaLocalizationConsumer.deactivate();
        Log.v("VisionThread", "Exited due to opMode no longer being active.");
    }

    public RealMatrix getVuforiaMatrix() {
        return vuforiaLocalizationConsumer.getVuforiaMatrix();
    }
}