package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    private final TeamMarkerDetector teamMarkerDetector;

//    private final CargoDetectorConsumer cargoDetectorConsumer;
//    private Thread cargoDetectionThread;

    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final Robot robot;
    private ManagedCamera managedCamera;
    private long updateTime = 0;
    private long lastLoopTime = 0;

    public VisionThread(Robot robot, WebcamName camera) {
        this.robot = robot;
//        this.cargoDetectorConsumer = new CargoDetectorConsumer(robot, PhysicalCamera.of(robot));
        this.vuforiaLocalizationConsumer = new VuforiaLocalizationConsumer(robot, PhysicalCamera.of(robot), camera, robot.hardwareMap);
        this.teamMarkerDetector = new TeamMarkerDetector();
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

            this.managedCamera = new ManagedCamera(
                    robot.camera,
                    vuforiaLocalizationConsumer,
                    openCVDumper,
                    getTeamMarkerDetector()
//                    cargoDetectorConsumer
            );

//            cargoDetectionThread = new Thread(cargoDetectorConsumer);
//            cargoDetectionThread.start();

            robot.telemetryDump.registerTelemeter(this);
//            robot.telemetryDump.registerTelemeter(cargoDetectorConsumer);

            Log.v("VisionThread", "Done initing camera");

            while (robot.running()) {
                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;
            }
            this.vuforiaLocalizationConsumer.deactivate();
            this.managedCamera.close();
            Log.v("VisionThread", "Exited due to opMode's no longer being active.");
        } catch (Exception e) {
            if (robot.isOpModeActive()) // if we got interrupted bc the opmode is stopping its fine;  if we're still running, rethrow
                throw e;
        } finally {
            /*if(cargoDetectionThread != null)
                cargoDetectionThread.interrupt();*/
            if (managedCamera != null)
                managedCamera.close();
        }
    }

    public RealMatrix getVuforiaMatrix() {
        if(managedCamera == null || !managedCamera.vuforiaActive)
            return null;

        return vuforiaLocalizationConsumer.getLocationRealMatrix();
    }

    public ManagedCamera getManagedCamera() {
        return this.managedCamera;
    }

/*    public ConcurrentHashMap<Point, Classifier.Recognition> getDetectedGameElements() {
        return cargoDetectorConsumer.getDetectedGameElements();
    }*/

    public double getCameraAngle() {
        return vuforiaLocalizationConsumer.getCameraAngle();
    }

    public TeamMarkerDetector getTeamMarkerDetector() {
        return teamMarkerDetector;
    }
}