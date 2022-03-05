package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.DashboardUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Collectors;

public class TelemetryDump implements PoseWatcher {
    private final Telemetry telemetry;

    private final ConcurrentLinkedQueue<Telemeter> telemeters = new ConcurrentLinkedQueue<>();
    private final FtcDashboard dashboard;

    private final List<Pose> poseHistory = new ArrayList<>();

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public void removeTelemeter(Telemeter telemeter) {
        telemeters.remove(telemeter);
    }

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;

        this.dashboard = FtcDashboard.getInstance();
        this.dashboard.setTelemetryTransmissionInterval(25);
    }

    public void update() {
        StringBuilder update = new StringBuilder();

        List<Integer> indexes = telemeters.stream().map(Telemeter::getShowIndex).sorted().collect(Collectors.toList());
        for (int i : indexes) {
            List<Telemeter> index = telemeters.stream().filter(telemeter -> telemeter.getShowIndex() == i).collect(Collectors.toList());
            update.append(getData(index));
        }

        telemetry.addLine(update.toString());
        telemetry.update();
    }

    @Override
    public void sendPose(Pose pose) {
        if (Robot.DEBUG) {
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

    private String getData(List<Telemeter> telemetors) {
        StringBuilder msg = new StringBuilder();

        for (Telemeter telemeter : telemetors) {
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

        return msg.append("\n\n").toString();
    }
}