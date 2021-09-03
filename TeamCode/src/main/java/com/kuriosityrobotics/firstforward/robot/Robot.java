package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.telemetry.TelemetryDump;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public final static boolean DEBUG = false;
    private final static String configLocation = "configurations/mainconfig.toml";


    private final Thread[] threads;

    public final TelemetryDump telemetryDump;

    public final HardwareMap hardwareMap;
    public final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;
        telemetryDump = new TelemetryDump(telemetry, DEBUG);

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 173");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
            linearOpMode.stop();
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }

        threads = new Thread[]{
                new Thread(new SensorThread(this, configLocation)),
                new Thread(new ModuleThread(this))
        };
    }

    public void start() {
        for (Thread thread : threads) {
            thread.start();
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean running() {
        return isOpModeActive();
    }
}
