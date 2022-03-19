package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;

import java.util.ArrayList;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final Module[] modules;

    private boolean started = false;

    private long updateDuration = 0;
    private long timeOfLastUpdate = 0;

    public ModuleThread(Robot robot, Module[] modules) {
        this.robot = robot;
        this.modules = modules;

        robot.telemetryDump.registerTelemeter(this);
    }

    /**
     * Calls .update() on all modules and telemetryDump while `robot.running()` is true.
     */
    public void run() {
        while (robot.running()) {
            if (!started && robot.started()) {
                for (Module module : modules) {
                    if (module.isOn()) {
                        module.onStart();
                    }
                }

                started = true;
            }

            for (Module module : modules) {
                if (module.isOn()) {
                    module.update();
                }
            }

            robot.telemetryDump.update();

            long currentTime = SystemClock.elapsedRealtime();
            updateDuration = currentTime - timeOfLastUpdate;
            timeOfLastUpdate = currentTime;
        }

        for (Module module : modules) {
            if (module.isOn()) {
                module.onClose();
            }
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

        data.add("Update time: " + updateDuration);

        return data;
    }

    @Override
    public int getShowIndex() {
        return Telemeter.super.getShowIndex(); // which is 2
    }
}