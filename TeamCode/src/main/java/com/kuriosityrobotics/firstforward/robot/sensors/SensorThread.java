package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.configuration.Configurator;

import java.util.ArrayList;

public class SensorThread implements Runnable {
    private final String configLocation;
    private final ArrayList<SensorTick> ticks = new ArrayList<>();

    public SensorThread(String configLocation) {
        this.configLocation = configLocation;
    }

    public void registerTick(SensorTick tick) {
        this.ticks.add(tick);
    }

    @Override
    public void run() {
        Configurator.loadConfig(configLocation, ticks.toArray());
    }
}
