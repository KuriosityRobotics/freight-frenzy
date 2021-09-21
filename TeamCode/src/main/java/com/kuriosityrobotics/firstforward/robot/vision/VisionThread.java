package com.kuriosityrobotics.firstforward.robot.vision;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

// TODO: Turn this into a thread
public class VisionThread implements Runnable, Telemeter {
    private List<LocalizationConsumer> localizationConsumers;

    public final ManagedCamera managedCamera;
    private final Robot robot;

    private final String webcamName;

    public VisionThread(Robot robot, List<LocalizationConsumer> localizationConsumers, String webcamName) {
        this.robot = robot;
        this.webcamName = webcamName;
        robot.telemetryDump.registerTelemeter(this);

        this.localizationConsumers = localizationConsumers;
        List<VuforiaConsumer> vuforiaConsumers = localizationConsumers
                .stream()
                .map(e -> (VuforiaConsumer) e)
                .collect(Collectors.toList());
        this.managedCamera = new ManagedCamera(webcamName, robot.hardwareMap, vuforiaConsumers);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> telemetryData = new ArrayList<>();
        for (LocalizationConsumer consumer: localizationConsumers) {
            telemetryData.addAll(consumer.logPositionandDetection());
        }
        return telemetryData;
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
            Log.v("VisionThread", "Managed Cameras are running :)");
        }

        this.localizationConsumers.forEach(LocalizationConsumer::deactivate);
        Log.v("VisionThread", "Exited due to opMode no longer being active.");
    }
}