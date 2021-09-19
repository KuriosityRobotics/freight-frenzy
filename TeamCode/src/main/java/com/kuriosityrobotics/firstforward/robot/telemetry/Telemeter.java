package com.kuriosityrobotics.firstforward.robot.telemetry;

import java.util.ArrayList;

public interface Telemeter {

    default Iterable<String> getTelemetryData() {
        return new ArrayList<>();
    }

    String getName();

    boolean isOn();
}
