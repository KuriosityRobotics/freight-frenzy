package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryDump {
    private final Telemetry telemetry;

    private final ArrayList<Telemeter> telemeters = new ArrayList<>();

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        StringBuilder msg = new StringBuilder();

        for (Telemeter telemeter : telemeters) {
            if (telemeter.isOn()) {
                // ---Name---\n
                msg.append("---").append(telemeter.getName()).append("---\n");

                for (String line : telemeter.getTelemetryData()) {
                    // telemetry_line\n
                    msg.append(line).append("\n");
                }

                // newline for every section
                msg.append("\n");
            }
        }

        telemetry.addLine(msg.toString());
        telemetry.update();
    }
}
