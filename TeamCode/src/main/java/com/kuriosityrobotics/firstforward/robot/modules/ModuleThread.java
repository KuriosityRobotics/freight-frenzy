package com.kuriosityrobotics.firstforward.robot.modules;

import static java.text.MessageFormat.format;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AsynchProcess;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.Executor;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final HashMap<Module, AsynchProcess> modules;

    private boolean started = false;

    public ModuleThread(Robot robot, Module... modules) {
        this.robot = robot;
        this.modules = new HashMap<>(modules.length);
        for (Module module : modules)
            if (module.maxFrequency() == 0)
                this.modules.put(module, AsynchProcess.blocking(module::update));
            else
                this.modules.put(module, AsynchProcess.blocking(module::update, module.maxFrequency()));


        robot.getTelemetryDump().registerTelemeter(this);
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