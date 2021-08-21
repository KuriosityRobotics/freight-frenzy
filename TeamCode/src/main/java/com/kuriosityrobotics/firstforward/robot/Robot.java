package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.configuration.Configurator;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;

import java.io.IOException;

public class Robot {
    public final static boolean WILL_FILE_DUMP = true;
    private final static String configLocation = "configurations/mainconfig.toml";

    private Module[] modules;

    private Thread[] threads;
    private ModuleThread moduleThread;

    public Robot() {
        modules = new Module[]{};

        start();
    }

    public void start() {
        threads = new Thread[]{
                new Thread(new ModuleThread(this))
        };

        for (Thread thread : threads) {
            thread.start();
        }
    }

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
