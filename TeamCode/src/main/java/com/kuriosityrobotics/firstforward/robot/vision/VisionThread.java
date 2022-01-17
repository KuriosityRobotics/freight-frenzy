package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetection;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.HashMap;

public class VisionThread implements Runnable, Telemeter {
    public WebcamName activeCamera;

    private VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private TeamMarkerDetection teamMarkerDetector;
    private final OpenCVDumper openCVDumper;

    public ManagedCamera managedCamera;
    private final Robot robot;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, VuforiaLocalizationConsumer vuforiaLocalizationConsumer) {
        this.robot = robot;

        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;
        openCVDumper = new OpenCVDumper(robot.isDebug());
        this.managedCamera = new ManagedCamera(robot.leftCamera, robot.frontCamera, vuforiaLocalizationConsumer, openCVDumper);

        this.activeCamera = robot.leftCamera;

        robot.telemetryDump.registerTelemeter(this);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
        telemetryData.addAll(vuforiaLocalizationConsumer.logPositionandDetection());
        telemetryData.add("Team marker location: " + teamMarkerDetector.getLocation());
        return telemetryData;
    }

//    @Override
//    public HashMap<String, Object> getDashboardData() {
//        HashMap<String, Object> data = new HashMap<>();
//        data.put("Vision Thread Update time: ", updateTime);
//        data.put("Robot Location: ", this.vuforiaLocalizationConsumer.getFormattedMatrix());
//        return data;
//    }

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
            if (!activeCamera.equals(managedCamera.getActiveCameraName()))
                managedCamera.activateCamera(activeCamera);

            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        this.vuforiaLocalizationConsumer.deactivate();
        Log.v("VisionThread", "Exited due to opMode no longer being active.");
    }
}