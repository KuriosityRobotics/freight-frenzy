package com.kuriosityrobotics.firstforward.robot.telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

public interface Telemeter {

    default Iterable<String> getTelemetryData() {
        return new ArrayList<>();
    }

    String getName();

    boolean isOn();
}
