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
    public WebcamLocalization(Robot robot) {
        robot.telemetryDump.registerTelemeter(this);

        this.vuforiaConsumer = new LocalizationConsumer();
        ManagedCamera vuMarkCam = new ManagedCamera("Webcam 1", robot.hardwareMap, this.vuforiaConsumer);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Robot Coordinates: " + this.vuforiaConsumer.getPosition().toString());
        data.add("Robot Orientation: " + this.vuforiaConsumer.getRobotOrientation().toString());
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
}