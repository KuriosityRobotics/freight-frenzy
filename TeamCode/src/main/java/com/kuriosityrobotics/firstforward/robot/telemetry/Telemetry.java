package com.kuriosityrobotics.firstforward.robot.telemetry;

import java.util.ArrayList;

public class Telemetry {
    private static final ArrayList<Telemeter> telemeters = new ArrayList<>();

    public static void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }
}
