package com.kuriosityrobotics.firstforward.robot.telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

public interface Telemeter {
    /**
     * Returns the fields to be telemetried.  By default, returns all public fields.
     * @return
     */
    default Map<String, Object> getDataFields() {
        return Arrays.stream(getClass().getDeclaredFields()) // cursed
                .filter(n -> Modifier.isPublic(n.getModifiers()))
                .collect(Collectors.toMap(Field::getName, n -> {
            try {
                return n.get(this);
            } catch (IllegalAccessException e) {
                return null;
            }
        }));
    }
}
