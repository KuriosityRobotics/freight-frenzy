package com.kuriosityrobotics.firstforward.robot.debug;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;

import java.util.ArrayList;

public class DebugThread implements Runnable, Telemeter {
    private final Robot robot;
    private final boolean isOn;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public DebugThread(Robot robot, Boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        FileDump.activate();
    }

    @Override
    public void run() {
        while (robot.running() && isOn) {
            FileDump.update();

            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);

        return data;
    }

    @Override
    public String getName() {
        return "DebugThread";
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    public void onClose() {
        FileDump.close();
    }
}
