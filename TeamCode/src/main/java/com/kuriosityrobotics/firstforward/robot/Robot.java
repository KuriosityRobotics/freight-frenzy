package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.debug.DebugThread;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.CarouselModule;
import com.kuriosityrobotics.firstforward.robot.modules.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.modules.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.vision.VisionThread;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {
    private static final boolean DEBUG = false;
    private static final String configLocation = "configurations/mainconfig.toml";

    private Thread[] threads;
    private final Module[] modules;

    public final SensorThread sensorThread;
    public final ModuleThread moduleThread;
    public final VisionThread visionThread;
    public final DebugThread debugThread;

    public final Drivetrain drivetrain;
    public final IntakeModule intakeModule;
    public final OuttakeModule outtakeModule;
    public final CarouselModule carouselModule;

    public TelemetryDump telemetryDump;

    public LocalizationConsumer localizationConsumer;

    public final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode, Pose pose) throws RuntimeException {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        telemetryDump = new TelemetryDump(telemetry, DEBUG);

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }

        // modules
        drivetrain = new Drivetrain(this);
        intakeModule = new IntakeModule(this, true);
        outtakeModule = new OuttakeModule(this);
        carouselModule = new CarouselModule(this);

        modules = new Module[]{
                drivetrain,
                intakeModule,
                outtakeModule,
                carouselModule
        };

        localizationConsumer = new LocalizationConsumer();

        // threads
        sensorThread = new SensorThread(this, configLocation, localizationConsumer, pose);
        moduleThread = new ModuleThread(this, this.modules);
        visionThread = new VisionThread(this, localizationConsumer, "Webcam 1");
        debugThread = new DebugThread(this, DEBUG);

        this.start();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) throws Exception {
        this(hardwareMap, telemetry, linearOpMode, new Pose(0.0, 0.0, 0.0));
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

    public Servo getServo(String name) {
        try {
            return hardwareMap.servo.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Servo with name " + name + " could not be found. Exception: " + exception);
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

    public boolean isDebug() {
        return DEBUG;
    }
}
