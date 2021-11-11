package com.kuriosityrobotics.firstforward.robot;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.DebugThread;
import com.kuriosityrobotics.firstforward.robot.modules.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.vision.VisionThread;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {
    private static final boolean DEBUG = false;
    private static final String configLocation = "configurations/mainconfig.toml";

    private Thread[] threads;
    private final Module[] modules;

    private final SensorThread sensorThread;
    private final ModuleThread moduleThread;
    private final VisionThread visionThread;
    private final DebugThread debugThread;

    public final Drivetrain drivetrain;
    public TelemetryDump telemetryDump;

    public LocalizationConsumer localizationConsumer;

    public final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
//    public final LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) throws Exception {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;
        try {
            telemetryDump = new TelemetryDump(telemetry, DEBUG);
        } catch (NullPointerException e) {
            Log.v("Robot", "No telemetry provided. More info: " + e);
        }

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
//            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new Exception("One or more of the REV hubs could not be found. More info: " + e);
        }

        drivetrain = new Drivetrain(this);

        modules = new Module[]{
                drivetrain
        };

        localizationConsumer = new LocalizationConsumer();

        sensorThread = new SensorThread(this, configLocation, localizationConsumer);
        moduleThread = new ModuleThread(this, this.modules);
        visionThread = new VisionThread(this, localizationConsumer, "Webcam 1");
        debugThread = new DebugThread(this);

        start();
    }

    public void start() {
        threads = new Thread[]{
                new Thread(sensorThread),
                new Thread(moduleThread),
                new Thread(visionThread),
                new Thread(debugThread)
        };

        for (Thread thread : threads) {
            thread.start();
        }
    }

    public DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor with name " + name + " could not be found. Exception: " + exception);
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean running() {
        return (!linearOpMode.isStopRequested() && !linearOpMode.isStarted()) || isOpModeActive();
    }

    public boolean started() {
        return linearOpMode.isStarted();
    }

    public SensorThread getSensorThread() {
        return this.sensorThread;
    }
}
