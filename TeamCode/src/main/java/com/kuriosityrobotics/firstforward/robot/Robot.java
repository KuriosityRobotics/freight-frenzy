package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.telemetry.TelemetryDump;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javassist.NotFoundException;

public class Robot {
    private static final boolean DEBUG = false;
    private static final String configLocation = "configurations/mainconfig.toml";

    private final Thread[] threads;
    private final Module[] modules;

    public final TelemetryDump telemetryDump;

    public final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) throws NotFoundException {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;
        telemetryDump = new TelemetryDump(telemetry, DEBUG);

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new NotFoundException("One or more of the REV hubs could not be found. More info: " + e);
        }

        modules = new Module[]{};
        threads = new Thread[]{
                new Thread(new SensorThread(this, configLocation)),
                new Thread(new ModuleThread(this, this.modules))
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
        return (!linearOpMode.isStopRequested() && !linearOpMode.isStarted()) || isOpModeActive();
    }
}
