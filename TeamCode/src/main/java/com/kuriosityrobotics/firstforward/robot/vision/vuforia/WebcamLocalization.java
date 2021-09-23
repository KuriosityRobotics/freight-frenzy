package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

/**
 * Webcam localization module
 */
public class WebcamLocalization implements Telemeter {
    private LocalizationConsumer vuforiaConsumer;
    private Point robotCoordinates;
    private Orientation robotRotation;
    public WebcamLocalization(Robot robot) {
        robot.telemetryDump.registerTelemeter(this);

        this.vuforiaConsumer = new LocalizationConsumer();
        ManagedCamera vuMarkCam = new ManagedCamera("Webcam 1", robot.hardwareMap, this.vuforiaConsumer);
        this.robotCoordinates = vuforiaConsumer.getPosition();
        this.robotRotation = vuforiaConsumer.getRobotOrientation();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        Point currentPos = this.vuforiaConsumer.getPosition();
        Orientation currentOrientation = this.vuforiaConsumer.getRobotOrientation();

        if (currentPos != null) {
            data.add("Robot Coordinates: " + currentPos.toString());
        }
        else {
            data.add("Robot Coordinates: UKNOWN");
        }

        if (currentOrientation != null) {
            data.add("Robot Orientation: " + currentOrientation.toString());
        }
        else {
            data.add("Robot Orientation: UKNOWN");
        }

        return data;
    }

    public Point getRobotCoordinates() {
        return this.robotCoordinates;
    }

    public Orientation getRobotRotation() {
        return this.robotRotation;
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