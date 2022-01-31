package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.DashboardUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class TelemetryDump implements PoseWatcher {
    private final Telemetry telemetry;
    private final boolean debug;

    private final Collection<Telemeter> telemeters = new ConcurrentLinkedQueue<>();
    public FtcDashboard dashboard;

    private final List<Pose> poseHistory = new ArrayList<>();

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public void removeTelemeter(Telemeter telemeter) {
        telemeters.remove(telemeter);
    }

    public TelemetryDump(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;

        this.dashboard = FtcDashboard.getInstance();
        this.dashboard.setTelemetryTransmissionInterval(25);
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

    @Override
    public void sendPose(Pose pose) {
        if (debug) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            for (Telemeter telemeter : telemeters) {
                if (telemeter.getDashboardData() != null) {
                    for (Map.Entry<String, Object> entry : telemeter.getDashboardData().entrySet()) {
                        packet.put(entry.getKey(), entry.getValue());
                    }
                }
            }

            Pose dashboardPose = pose.toFTCSystem();

            poseHistory.add(dashboardPose);
            DashboardUtil.drawRobot(canvas, dashboardPose);
            DashboardUtil.drawPoseHistory(canvas, poseHistory);

            dashboard.sendTelemetryPacket(packet);
        }
    }

//    private Set<Map.Entry<String, Object>> getAllFields(Telemeter telemeter) {
//        return Arrays.stream(telemeter.getClass().getDeclaredFields()) // cursed
//                .filter(n -> Modifier.isPublic(n.getModifiers()))
//                .collect(Collectors.toMap(Field::getName, n -> {
//                    try {
//                        return n.get(telemeter);
//                    } catch (IllegalAccessException e) {
//                        return null;
//                    }
//                })).entrySet();
//    }
}