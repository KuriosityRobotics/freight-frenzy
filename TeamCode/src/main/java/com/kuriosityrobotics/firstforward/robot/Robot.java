package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.configuration.Configurator;
import com.kuriosityrobotics.firstforward.robot.actions.ActionsThread;
import com.kuriosityrobotics.firstforward.robot.actions.ClawState;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.vision.VisionThread;

import java.io.IOException;

public class Robot {
    private final static String configLocation = "configurations/mainconfig.toml";

    private final static Runnable[] modules = {
            new ActionsThread(configLocation),
            new SensorThread(configLocation),
            new VisionThread(configLocation)
    };
    public static void run() throws IOException {
        Configurator.loadConfigFieldsStatic(configLocation, "com.kuriosityrobotics");
        System.out.println(new ClawState(null, null).CLAW_DISTANCE);
    }

    public static void main(String[] args) throws IOException {
        run();
    }
}
