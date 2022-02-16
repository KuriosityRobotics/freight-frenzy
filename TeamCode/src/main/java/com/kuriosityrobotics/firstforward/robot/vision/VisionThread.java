package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    public final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    public TeamMarkerDetector teamMarkerDetector;
    public OpenCVDumper openCVDumper;

    public SingleManagedCamera singleManagedCamera;
    private final Robot robot;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, WebcamName camera) {
        this.robot = robot;

        this.vuforiaLocalizationConsumer = new VuforiaLocalizationConsumer(robot, camera, robot.hardwareMap);
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
        try {
            this.openCVDumper = new OpenCVDumper(robot.isDebug());
            this.teamMarkerDetector = new TeamMarkerDetector();
            CargoDetectorConsumer cargoDetector = new CargoDetectorConsumer(robot);
            this.singleManagedCamera = new SingleManagedCamera(robot.camera, vuforiaLocalizationConsumer, openCVDumper, teamMarkerDetector, cargoDetector);

            Log.v("VisionThread", "Done initing camera");

            robot.telemetryDump.registerTelemeter(this);
            robot.telemetryDump.registerTelemeter(cargoDetector);

            while (robot.running()) {
                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;
            }
            this.vuforiaLocalizationConsumer.deactivate();
            this.singleManagedCamera.close();
            Log.v("VisionThread", "Exited due to opMode's no longer being active.");
        } catch (Exception e) {
            if (robot.isOpModeActive()) // if we got interrupted bc the opmode is stopping its fine;  if we're still running, rethrow
                throw e;
        } finally {
            if (singleManagedCamera != null)
                singleManagedCamera.close();
        }
    }

    public RealMatrix getVuforiaMatrix() {
        return vuforiaLocalizationConsumer.getLocationRealMatrix();
    }
}