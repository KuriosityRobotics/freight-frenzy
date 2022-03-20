package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;
import android.util.Pair;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;

import java.util.ArrayList;
import java.util.List;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final Module[] modules;

    private boolean started = false;

    private long updateDuration = 0;
    private final List<Pair<String, Long>> moduleUpdateTimes;

    public ModuleThread(Robot robot, Module[] modules) {
        this.robot = robot;
        this.modules = modules;

        robot.telemetryDump.registerTelemeter(this);
        moduleUpdateTimes = new ArrayList<>(5);
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

            moduleUpdateTimes.clear();
            for (Module module : modules) {
                if (module.isOn()) {
                    long start = SystemClock.elapsedRealtime();
                    module.update();
                    moduleUpdateTimes.add(new Pair<>(module.getName(), SystemClock.elapsedRealtime() - start));
                }
            }
            Log.v("modules", "module update times: " + moduleUpdateTimes);

            robot.telemetryDump.update();

            long currentTime = SystemClock.elapsedRealtime();
            updateDuration = currentTime - overallStart;
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
            int moduleUpdate = 0;
            for (int i = 0; i < moduleUpdateTimes.size(); i++) {
                data.add(moduleUpdateTimes.get(i).first + " Update time: " + moduleUpdateTimes.get(i).second);
                moduleUpdate += moduleUpdateTimes.get(i).second;
            }
            data.add("Update Time not including modules: " + (updateDuration - moduleUpdate));

            return data;
        }
    }
}