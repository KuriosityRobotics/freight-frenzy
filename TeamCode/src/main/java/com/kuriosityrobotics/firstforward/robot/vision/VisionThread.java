package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import java.util.ArrayList;
import java.util.HashMap;

public class VisionThread implements Runnable, Telemeter {
    private VuforiaLocalizationConsumer vuforiaLocalizationConsumer;

    private ManagedCamera managedCamera;
    private final Robot robot;

    private final OpenCVDumper openCVDumper;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, VuforiaLocalizationConsumer vuforiaLocalizationConsumer) {
        this.robot = robot;
        robot.telemetryDump.registerTelemeter(this);
        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;
        openCVDumper = new OpenCVDumper(robot.isDebug());
        this.managedCamera = new ManagedCamera(robot.cameraName1, robot.cameraName2, vuforiaLocalizationConsumer, openCVDumper);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
        telemetryData.addAll(vuforiaLocalizationConsumer.logPositionandDetection());
        return telemetryData;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("Vision Thread Update time: ", updateTime);
        data.put("Robot Location: ", this.vuforiaLocalizationConsumer.getFormattedMatrix());
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
        while (robot.running()) {
            try {
                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;

                if (robot.sensorThread.getPose().heading < 3 * Math.PI / 4 && robot.sensorThread.getPose().heading > -Math.PI / 2) {
                    managedCamera.activateCamera(robot.cameraName1);
                } else {
                    managedCamera.activateCamera(robot.cameraName2);
                }

                Thread.sleep(100);
            } catch (InterruptedException e) {
                Log.e("VisionThread", "Thread Interupted: ", e);
            }
        }
        this.vuforiaLocalizationConsumer.deactivate();
        Log.v("VisionThread", "Exited due to opMode no longer being active.");
    }
}