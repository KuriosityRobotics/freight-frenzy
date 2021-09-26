package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.vuforia.Trackable;

import java.util.ArrayList;

/**
 * Webcam localization module
 */
public class WebcamLocalization implements Telemeter {
    private LocalizationConsumer vuforiaConsumer;
    private VuforiaConsumer[] vuforiaConsumers;

    private final ManagedCamera managedCamera;
    private Trackable detectedTrackable;
    private Point trackableLocation;

    public WebcamLocalization(Robot robot) {
        robot.telemetryDump.registerTelemeter(this);

        this.vuforiaConsumer = new LocalizationConsumer();
        this.vuforiaConsumers = new VuforiaConsumer[0];
        this.vuforiaConsumers[0] = this.vuforiaConsumer;
        this.managedCamera = new ManagedCamera("Webcam 1", robot.hardwareMap, this.vuforiaConsumers);
    }

    public void stopConsuming() {
        this.vuforiaConsumer.deactivate();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        return this.vuforiaConsumer.logPositionandDetection();
    }

    @Override
    public String getName() {
        return "WebcamLocalization";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    public Trackable getTrackable() {
        return this.detectedTrackable;
    }

    public Point getTrackableLocation() {
        return this.trackableLocation;
    }
}