package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final Module[] modules;

    private boolean started = false;

    private long updateDuration = 0;
    private Map<Module, Long> moduleUpdateTimes;

    public ModuleThread(Robot robot, Module[] modules) {
        this.robot = robot;
        this.modules = modules;

        robot.telemetryDump.registerTelemeter(this);
        moduleUpdateTimes = new HashMap<>(5);
    }

    /**
     * Calls .update() on all modules and telemetryDump while `robot.running()` is true.
     */
    public void run() {
        while (robot.running()) {
            long overallStart = SystemClock.elapsedRealtime();
            if (!started && robot.started()) {
                for (Module module : modules) {
                    if (module.isOn()) {
                        module.onStart();
                    }
                }

                started = true;
            }

            Map<Module, Long> aTime = new HashMap<>(5);
            for (Module module : modules) {
                if (module.isOn()) {
                    long start = SystemClock.elapsedRealtime();
                    module.update();
                    aTime.put(module, SystemClock.elapsedRealtime() - start);
                }
            }

            robot.telemetryDump.update();
            moduleUpdateTimes = aTime;
            updateDuration = SystemClock.elapsedRealtime() - overallStart;
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
        synchronized (this) {
            ArrayList<String> data = new ArrayList<>();

            data.add("Overall Update time: " + updateDuration);
            long moduleUpdate = 0; // what is this
            for (Map.Entry<Module, Long> entry : moduleUpdateTimes.entrySet()) { // so much cleaner compared to foreach
                Module module = entry.getKey();
                Long time = entry.getValue();
                data.add(module.getName() + "Update Time: " + time);
                moduleUpdate += time;
            }

            data.add("Update Time not including modules: " + (updateDuration - moduleUpdate));

            return data;
        }
    }
}