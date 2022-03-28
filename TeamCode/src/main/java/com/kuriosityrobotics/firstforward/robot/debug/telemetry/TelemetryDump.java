package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.DashboardUtil;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Collectors;

public class TelemetryDump implements PoseWatcher {
    private final Telemetry telemetry;

    private final ConcurrentLinkedQueue<Telemeter> telemeters = new ConcurrentLinkedQueue<>();
    private final FtcDashboard dashboard;
    private final List<Pose> poseHistory = new ArrayList<>();
    private String alert;

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
        alert = null;

        this.dashboard = FtcDashboard.getInstance();
        this.dashboard.setTelemetryTransmissionInterval(25);
    }

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public void removeTelemeter(Telemeter telemeter) {
        telemeters.remove(telemeter);
    }

    public void update() {
        telemetry.addLine(getData(telemeters));
        telemetry.update();
    }

    @Override
    public void sendPose(Pose pose) {
        if (Robot.DEBUG) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            for (Telemeter telemeter : telemeters) {
                if (telemeter.getDashboardData() != null) {
                    packet.putAll(telemeter.getDashboardData());
                }
            }

            Pose dashboardPose = pose.toFTCSystem();

            poseHistory.add(dashboardPose);
            DashboardUtil.drawRobot(canvas, dashboardPose);
            DashboardUtil.drawPoseHistory(canvas, poseHistory);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void setAlert(String alert) {
        this.alert = alert;
    }

    public void clearAlert() {
        this.alert = null;
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

    private String getData(ConcurrentLinkedQueue<Telemeter> telemetors) {
        StringBuilder stringBuilder = new StringBuilder();

        if (alert != null)
            stringBuilder.append(alert).append("\n \n");

        var a = telemetors.stream().sorted(Comparator.comparing(Telemeter::getShowIndex))
                .filter(Telemeter::isOn)
                .map(telemeter ->
                        stringBuilder
                                .append("---")
                                .append(telemeter.getName())
                                .append("---\n")
                                // please java SHUT THE GELL UP I DON'T WANT TO USE STRING.JOIN
                                .append(telemeter.getTelemetryData().stream().collect(Collectors.joining("\n")))
                                .append("\n\n")
                ).collect(Collectors.toList()).toString();

        return stringBuilder.toString();
    }
}