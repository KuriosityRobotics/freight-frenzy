package com.kuriosityrobotics.firstforward.robot.vision;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.FreightDetectorConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class VisionThread implements Runnable, Telemeter {
    private final TeamMarkerDetector teamMarkerDetector;

    private final FreightDetectorConsumer freightDetectorConsumer;
    private Thread freightDetectionThread;

    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final Robot robot;
    private ManagedCamera managedCamera;
    private long updateTime = 0;
    private long lastLoopTime = 0;

    private boolean started = false;

    public VisionThread(Robot robot, WebcamName camera) {
        this.robot = robot;
        this.freightDetectorConsumer = new FreightDetectorConsumer(robot);
        this.teamMarkerDetector = new TeamMarkerDetector(robot);

        if (camera.isAttached()) {
            this.vuforiaLocalizationConsumer = new VuforiaLocalizationConsumer(robot, robot.getSensorThread().getKalmanFilter(), robot, robot.getHardwareMap());
        } else {
            this.vuforiaLocalizationConsumer = null;
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
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

            if (getVuforiaLocalizationConsumer() == null || !robot.running()) {
                return;
            }

            this.managedCamera = new
                    ManagedCamera(
                    robot.isUseCamera(),
                    getVuforiaLocalizationConsumer(),
                    robot, openCVDumper,
                    getTeamMarkerDetector(),
                    getFreightDetectorConsumer()
            );

            freightDetectionThread = new Thread(freightDetectorConsumer);
            freightDetectionThread.start();

            robot.getTelemetryDump().registerTelemeter(this);
            robot.getTelemetryDump().registerTelemeter(freightDetectorConsumer);

            Log.v("VisionThread", "Done initing camera");

            started = true;

            while (robot.running()) {
                if (robot.isTeleOp()) {
                    teamMarkerDetector.deactivate();
                }

                long currentTime = SystemClock.elapsedRealtime();
                updateTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;
            }

            this.getVuforiaLocalizationConsumer().deactivate();
            this.managedCamera.close();
            managedCamera = null;
            Log.v("VisionThread", "Exited due to opMode's no longer being active.");
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (freightDetectionThread != null)
                freightDetectionThread.interrupt();

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
        return getVuforiaLocalizationConsumer().getTargetCameraAngle();
    }

    public TeamMarkerDetector getTeamMarkerDetector() {
        return teamMarkerDetector;
    }

    public FreightDetectorConsumer getFreightDetectorConsumer() { return freightDetectorConsumer; }

    @Override
    protected void finalize() {
        if (managedCamera != null) {
            Log.w("VisionThread", "Camera closed in finalize()");
            managedCamera.close();
            managedCamera = null;
        }
    }

    public VuforiaLocalizationConsumer getVuforiaLocalizationConsumer() {
        return vuforiaLocalizationConsumer;
    }

    public boolean isStarted() {
        return started;
    }
}