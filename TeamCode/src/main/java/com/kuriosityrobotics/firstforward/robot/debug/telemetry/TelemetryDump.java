package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.DashboardUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class TelemetryDump implements PoseWatcher {
    private final Telemetry telemetry;
    private final boolean debug;

    private final ArrayList<Telemeter> telemeters = new ArrayList<>();
    public FtcDashboard dashboard;
    public TelemetryPacket packet;
    public Canvas canvas;

    private List<Pose> poseHistory = new ArrayList<>();

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public TelemetryDump(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;

        this.dashboard = FtcDashboard.getInstance();
        this.dashboard.setTelemetryTransmissionInterval(25);
        this.packet = new TelemetryPacket();
        this.canvas = new Canvas();
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

    public void sendPose(Pose pose) {
        Log.v("Dashboard", "Attempting to add pose...");
        poseHistory.add(pose);
        Log.v("Dashboard", "Pose successfully added");
        DashboardUtil.drawRobot(canvas, pose);
        Log.v("Dashboard", "Robot History Drawn");
        DashboardUtil.drawPoseHistory(canvas, poseHistory);
        Log.v("Dashboard", "Pose History Drawn");
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
