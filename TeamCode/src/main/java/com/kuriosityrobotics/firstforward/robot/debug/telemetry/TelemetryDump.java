package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class TelemetryDump {
    private final Telemetry telemetry;
    private final boolean debug;

    private final ArrayList<Telemeter> telemeters = new ArrayList<>();
    public FtcDashboard dashboard;
    public TelemetryPacket packet;

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public TelemetryDump(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;

        this.dashboard = FtcDashboard.getInstance();
        this.dashboard.setTelemetryTransmissionInterval(25);
        this.packet = new TelemetryPacket();
    }

    public void update() {
        StringBuilder msg = new StringBuilder();

        for (Telemeter telemeter : telemeters) {
            if (telemeter.isOn()) {
                // ---Name---\n
                msg.append("---").append(telemeter.getName()).append("---\n");

                if (debug) {
                    for (Map.Entry<String, Object> pair : getAllFields(telemeter)) {
                        // Key: Value \n
                        msg.append(pair.getKey()).append(": ").append(pair.getValue()).append("\n");
                    }
                } else {
                    for (String line : telemeter.getTelemetryData()) {
                        // telemetry_line\n
                        msg.append(line).append("\n");
                    }
                }

                // newline for every section
                msg.append("\n");
            }
        }

        telemetry.addLine(msg.toString());
        telemetry.update();

        parseDashboardData();
        dashboard.sendTelemetryPacket(packet);
    }

    public void parseDashboardData(){
        for(Telemeter telemeter : telemeters){
            if(telemeter.getDashboardData() != null) {
                for (String key : telemeter.getDashboardData().keySet()) {
                    sendDashboardData(key, telemeter.getDashboardData().get(key));
                }
            }
        }
    }

    public void sendDashboardData(String key, Object value){
        packet.put(key,value);
    }

    private Set<Map.Entry<String, Object>> getAllFields(Telemeter telemeter) {
        return Arrays.stream(telemeter.getClass().getDeclaredFields()) // cursed
                .filter(n -> Modifier.isPublic(n.getModifiers()))
                .collect(Collectors.toMap(Field::getName, n -> {
                    try {
                        return n.get(telemeter);
                    } catch (IllegalAccessException e) {
                        return null;
                    }
                })).entrySet();
    }
}
