package com.kuriosityrobotics.firstforward.robot.modules;

import static java.util.concurrent.ForkJoinPool.commonPool;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AsynchProcess;

import java.util.ArrayList;
import java.util.HashMap;

public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final HashMap<Module, AsynchProcess> modules;

    private boolean started = false;

    public ModuleThread(Robot robot, Module... modules) {
        this.robot = robot;
        this.modules = new HashMap<>(modules.length);
        for (Module module : modules)
            this.modules.put(module, AsynchProcess.blocking(module));


        robot.getTelemetryDump().registerTelemeter(this);
        this.modules.keySet().forEach(robot.getTelemetryDump()::registerTelemeter);
    }

    /**
     * Calls .update() on all modules and telemetryDump while `robot.running()` is true.
     */
    public void run() {
        while (robot.running()) {
            if (!started && robot.started()) {
                for (Module module : modules.keySet())
                    if (module.isOn())
                        module.onStart();
                started = true;
            }

            for (var module : modules.entrySet())
                if (module.getKey().isOn())
                    module.getValue().update();


            robot.getTelemetryDump().update();
        }

        for (Module module : modules.keySet()) {
            if (module.isOn()) {
                module.onClose();
            }
        }
        commonPool().shutdown();
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

            for (var module : modules.entrySet())
                data.add(module.getKey().getName() + " " + module.getValue().toString());

            return data;
        }
    }
}