package com.kuriosityrobotics.firstforward.robot.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Map;

public class TelemetryDump {
    private final Telemetry telemetry;
    private boolean debug;

    private static final ArrayList<Telemeter> telemeters = new ArrayList<>();

    public static void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public static void clearTelemeters() {
        telemeters.clear();
    }

    public TelemetryDump(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;
    }

    public void update() {
        StringBuilder msg = new StringBuilder();

        for (Telemeter telemeter : telemeters) {
            if (telemeter.isOn()) {
                // ---Name---\n
                msg.append("---").append(telemeter.getName()).append("---\n");

                if (debug) {
                    for (Map.Entry<String, Object> pair : telemeter.getDataFields().entrySet()) {
                        // Key: Value \n
                        msg.append(pair.getKey()).append(": ").append(pair.getValue()).append("\n");
                    }
                } else {
                    for (String line : telemeter.getTelemetryData()) {
                        // telemetry_line\n
                        msg.append(line).append("\n");
                    }
                }
            }
        }

        telemetry.addLine(msg.toString());
        telemetry.update();
    }
}
