package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.debug.DebugThread;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.modules.CarouselModule;
import com.kuriosityrobotics.firstforward.robot.modules.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.modules.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.vision.VisionThread;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Robot {
    private static final boolean DEBUG = false;
    private static final String configLocation = "configurations/mainconfig.toml";

    private Thread[] threads;
    private final Module[] modules;

    public final SensorThread sensorThread;
    public final ModuleThread moduleThread;
    public VisionThread visionThread;
    public final DebugThread debugThread;

    public final Drivetrain drivetrain;
    public final IntakeModule intakeModule;

    public final OuttakeModule outtakeModule;
    public final CarouselModule carouselModule;

    public TelemetryDump telemetryDump;

    public final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public WebcamName camera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) throws RuntimeException {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        this.camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetryDump = new TelemetryDump(telemetry, DEBUG);

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }

        // init sensorThread up here since drivetrain depends on it
        sensorThread = new SensorThread(this, configLocation);

        // modules
        drivetrain = new Drivetrain(this, sensorThread.getPose());
        intakeModule = new IntakeModule(this, true);
        outtakeModule = new OuttakeModule(this);
        carouselModule = new CarouselModule(this);

        modules = new Module[]{
                drivetrain,
                intakeModule,
                outtakeModule,
                carouselModule
        };

        // threads
        moduleThread = new ModuleThread(this, this.modules);

        telemetry.addData("> ", "Please wait for vuforia to init");
        telemetry.update();
        VuforiaLocalizationConsumer vuforiaLocalizationConsumer = new VuforiaLocalizationConsumer(this, camera, hardwareMap);
        visionThread = new VisionThread(this, vuforiaLocalizationConsumer);
        telemetry.addData("> ", "Vuforia has been initalized");
        telemetry.update();

        debugThread = new DebugThread(this, DEBUG);

        this.start();
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
