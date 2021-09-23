package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;

import java.util.ArrayList;

/**
 * Webcam localization module
 */
public class WebcamLocalization implements Telemeter {
    private LocalizationConsumer vuforiaConsumer;
    private final ManagedCamera managedCamera;

    public WebcamLocalization(Robot robot) {
        robot.telemetryDump.registerTelemeter(this);

        this.vuforiaConsumer = new LocalizationConsumer();
        this.managedCamera = new ManagedCamera("Webcam 1", robot.hardwareMap, this.vuforiaConsumer);
    }

    public void stopConsuming() {
        this.vuforiaConsumer.deactivate();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        return this.vuforiaConsumer.logPosition();
    }

    @Override
    public String getName() {
        return "WebcamLocalization";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}