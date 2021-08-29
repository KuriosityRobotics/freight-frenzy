package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.telemetry.TelemetryDump;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;

public class Robot {
    public final static boolean DEBUG = false;
    private final static String configLocation = "configurations/mainconfig.toml";

    private Module[] modules;

    private Thread[] threads;
    private ModuleThread moduleThread;

    public final TelemetryDump telemetryDump;

    public final HardwareMap hardwareMap;
    public final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        telemetryDump = new TelemetryDump(telemetry, DEBUG);
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        modules = new Module[]{};

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 173");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
            linearOpMode.stop();
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }

        start();
    }

    public void start() {
        threads = new Thread[]{
                new Thread(new SensorThread(this, configLocation)),
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

    @SuppressWarnings("unchecked") // intellij projecting
    public <T extends HardwareDevice> T getHardware(String name) { // generics poggers
        return (T) this.linearOpMode.hardwareMap.get(name);
    }
}
