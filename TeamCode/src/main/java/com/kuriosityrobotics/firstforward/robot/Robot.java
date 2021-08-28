package com.kuriosityrobotics.firstforward.robot;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.configuration.Configurator;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.telemetry.TelemetryDump;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;

public class Robot {
    public final static boolean DEBUG = false;
    private final static String configLocation = "configurations/mainconfig.toml";

    private Module[] modules;

    private Thread[] threads;
    private ModuleThread moduleThread;

    public final TelemetryDump telemetryDump;

    public final LinearOpMode linearOpMode;

    public Robot(Telemetry telemetry, LinearOpMode linearOpMode) {
        telemetryDump = new TelemetryDump(telemetry, DEBUG);
        this.linearOpMode = linearOpMode;

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
//        Configurator.runServer();
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

//        Log.v("robot", String.valueOf(linearOpMode.isStopRequested()));

        telemetryDump.update();
    }

    public boolean isStarted() {
        return linearOpMode.isStarted();
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isStopRequested() {
        return linearOpMode.isStopRequested();
    }
}
