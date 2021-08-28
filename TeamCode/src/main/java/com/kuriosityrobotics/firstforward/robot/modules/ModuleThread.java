package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;

import java.util.ArrayList;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable, Telemeter {
    final boolean SHOW_UPDATE_SPEED = true;

    Robot robot;

    private long updateTime = 0;
    private long lastUpdateTime = 0;

    public ModuleThread(Robot robot) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);
    }

    /**
     * Gets all modules from robot, then runs update on them.
     */
    public void run() {
        while (!robot.isStopRequested() && (!robot.isStarted() || robot.isOpModeActive())) {
            robot.update();

            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastUpdateTime;
            lastUpdateTime = currentTime;
        }

        Log.v("ModuleThread", "Exited due to opMode no longer being active.");
    }

    @Override
    public String getName() {
        return "ModuleThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);

        return data;
    }
}