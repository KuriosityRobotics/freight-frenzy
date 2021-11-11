package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;

import java.util.ArrayList;
import java.util.HashMap;

public class VisionThread implements Runnable, Telemeter {
    private LocalizationConsumer localizationConsumer;

    public ManagedCamera managedCamera;
    private final Robot robot;

    private final String webcamName;

    private final OpenCVDumper openCVDumper;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, LocalizationConsumer localizationConsumer, String webcamName) {
        this.robot = robot;
        this.webcamName = webcamName;
        robot.telemetryDump.registerTelemeter(this);
        this.localizationConsumer = localizationConsumer;
        openCVDumper = new OpenCVDumper();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
        telemetryData.addAll(localizationConsumer.logPositionandDetection());
        return telemetryData;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("Vision Thread Update time: ", updateTime);
        return data;
    }

    @Override
    public String getName() {
        return "WebcamLocalization";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public void run() {
        this.managedCamera = new ManagedCamera(webcamName, robot.hardwareMap, localizationConsumer, openCVDumper);

        while (robot.running()) {
            try {
                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;

                Thread.sleep(100);
            } catch (InterruptedException e) {
                Log.e("VisionThread", "Thread Interupted: ", e);
            }
        }
        this.localizationConsumer.deactivate();
        Log.v("VisionThread", "Exited due to opMode no longer being active.");
    }
}