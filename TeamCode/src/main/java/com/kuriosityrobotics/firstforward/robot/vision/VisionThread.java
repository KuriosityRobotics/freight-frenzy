package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    private final TeamMarkerDetector teamMarkerDetector;

    private final CargoDetectorConsumer cargoDetectorConsumer;
    private Thread cargoDetectionThread;

    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final Robot robot;
    private ManagedCamera managedCamera;
    private long updateTime = 0;
    private long lastLoopTime = 0;

    public boolean started = false;

    public VisionThread(Robot robot, WebcamName camera) {
        this.robot = robot;
        this.cargoDetectorConsumer = new CargoDetectorConsumer(robot, PhysicalCamera.of(robot));
        this.teamMarkerDetector = new TeamMarkerDetector();

        if (camera.isAttached()) {
            this.vuforiaLocalizationConsumer = new VuforiaLocalizationConsumer(robot, robot, PhysicalCamera.of(robot), camera, robot.hardwareMap);
        } else {
            this.vuforiaLocalizationConsumer = null;
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>(vuforiaLocalizationConsumer.logPositionAndDetection());
        telemetryData.add("Team marker location: " + getTeamMarkerDetector().getLocation());
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
            OpenCVDumper openCVDumper = new OpenCVDumper();

            if (vuforiaLocalizationConsumer == null || !robot.running()) {
                return;
            }

            this.managedCamera = new ManagedCamera(
                    robot.camera,
                    vuforiaLocalizationConsumer,
                    openCVDumper,
                    getTeamMarkerDetector(),
                    cargoDetectorConsumer
            );

            started = true;

            cargoDetectionThread = new Thread(cargoDetectorConsumer);
            cargoDetectionThread.start();

            robot.telemetryDump.registerTelemeter(this);
            robot.telemetryDump.registerTelemeter(cargoDetectorConsumer);

            Log.v("VisionThread", "Done initing camera");

            while (robot.running()) {
                if (robot.isOpModeActive())
                    teamMarkerDetector.deactivate();

                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;
            }

            this.vuforiaLocalizationConsumer.deactivate();
            this.managedCamera.close();
            managedCamera = null;
            Log.v("VisionThread", "Exited due to opMode's no longer being active.");
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if(cargoDetectionThread != null)
                cargoDetectionThread.interrupt();

            if (managedCamera != null) {
                managedCamera.close();
                managedCamera = null;
            }
        }
    }

/*    public ConcurrentHashMap<Point, Classifier.Recognition> getDetectedGameElements() {
        return cargoDetectorConsumer.getDetectedGameElements();
    }*/

    public double getCameraAngle() {
        return vuforiaLocalizationConsumer.getTargetCameraAngle();
    }

    public TeamMarkerDetector getTeamMarkerDetector() {
        return teamMarkerDetector;
    }

    @Override
    protected void finalize()  {
        if (managedCamera != null) {
            Log.w("VisionThread", "Camera closed in finalize()");
            managedCamera.close();
            managedCamera = null;
        }
    }
}