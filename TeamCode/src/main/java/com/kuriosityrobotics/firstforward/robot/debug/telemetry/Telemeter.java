
package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import java.util.ArrayList;
import java.util.HashMap;

public interface Telemeter {
    default Iterable<String> getTelemetryData() {
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
