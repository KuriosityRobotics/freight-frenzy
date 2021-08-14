package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.configuration.Configurator;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;

import java.io.IOException;

public class Robot {
    public final static boolean WILL_FILE_DUMP = true;

    private Module[] modules;

    private ModuleThread moduleThread;

    public Robot() {
        modules = new Module[]{};
    }

    public void start() {
        moduleThread = new ModuleThread(this);

        moduleThread.start();
    }

    private final static String configLocation = "configurations/mainconfig.toml";

    public static void run() throws IOException {
        Configurator.runServer();
    }

    public static void main(String[] args) throws IOException {
        run();
    }

    public void update() {
        for (Module module : modules) {
            if (module.isOn()) {
                module.update();
            }
        }
    }

    public boolean isOpModeActive() {
        return isOpModeActive();
    }
}
