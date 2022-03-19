
package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public interface Telemeter {
    default List<String> getTelemetryData() {
        return new ArrayList<>();
    }

    String getName();

    boolean isOn();

    default HashMap<String, Object> getDashboardData(){
        return new HashMap<>();
    }

    default int getShowIndex() {
        return 2;
    }
}
