package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final TeamMarkerDetector teamMarkerDetector;
    private final OpenCVDumper openCVDumper;

    public WebcamName activeCamera;
    public ManagedCamera managedCamera;
    private final Robot robot;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, VuforiaLocalizationConsumer vuforiaLocalizationConsumer) {
        this.robot = robot;

        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;
        this.openCVDumper = new OpenCVDumper(robot.isDebug());
        this.teamMarkerDetector = new TeamMarkerDetector();
        this.managedCamera = new ManagedCamera(robot.leftCamera, robot.frontCamera, vuforiaLocalizationConsumer, openCVDumper, teamMarkerDetector);
        this.managedCamera.activateCamera(robot.frontCamera);
        this.activeCamera = robot.leftCamera;

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

    private WebcamName getIdealVuforiaCamera() {
        if (robot.sensorThread.getPose().heading < (3 * Math.PI / 4) && robot.sensorThread.getPose().heading > (-Math.PI / 4))
            return robot.leftCamera;
        else
            return robot.frontCamera;
    }

    @Override
    public void run() {
        while (robot.running()) {
//            var idealCamera = getIdealVuforiaCamera();
//            if (!idealCamera.equals(managedCamera.getActiveCameraName()))
//                managedCamera.activateCamera(idealCamera);

            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        this.vuforiaLocalizationConsumer.deactivate();
        Log.v("VisionThread", "Stopped;  robot no longer running.");
    }
}