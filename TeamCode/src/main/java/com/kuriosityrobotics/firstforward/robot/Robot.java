package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.debug.DebugThread;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.modules.carousel.CarouselModule;
import com.kuriosityrobotics.firstforward.robot.modules.drivetrain.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.leds.LEDModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.VisionThread;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Robot implements LocationProvider {
    public static final boolean DEBUG = false;
    private static final String configLocation = "configurations/mainconfig.toml";

    public final SensorThread sensorThread;
    public final ModuleThread moduleThread;
    public VisionThread visionThread;
    public final DebugThread debugThread;

    public final Drivetrain drivetrain;
    public final IntakeModule intakeModule;

    public final OuttakeModule outtakeModule;
    public final CarouselModule carouselModule;

    public final LEDModule ledModule;

    public final TelemetryDump telemetryDump;

    public final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;

    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public final WebcamName camera;

    public static boolean isBlue = false;
    public static boolean isCarousel = false;
    public boolean isCamera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode, boolean isCamera) throws RuntimeException {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        this.camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetryDump = new TelemetryDump(telemetry);

        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }

        ActionExecutor.reset();

        // init sensorThread up here since drivetrain depends on it
        sensorThread = new SensorThread(this, configLocation);
        telemetryDump.registerTelemeter(sensorThread);

        // modules
        drivetrain = new Drivetrain(sensorThread.odometry, hardwareMap);
        telemetryDump.registerTelemeter(drivetrain);

        outtakeModule = new OuttakeModule(this, hardwareMap);
        telemetryDump.registerTelemeter(outtakeModule);

        intakeModule = new IntakeModule(hardwareMap, outtakeModule);
        telemetryDump.registerTelemeter(intakeModule);

        carouselModule = new CarouselModule(hardwareMap);
        telemetryDump.registerTelemeter(carouselModule);

        ledModule = new LEDModule(this);
        telemetryDump.registerTelemeter(ledModule);

        Module[] modules = new Module[]{
                drivetrain,
                intakeModule,
                outtakeModule,
                carouselModule,
                ledModule
        };

        // threads
        moduleThread = new ModuleThread(this, modules);

        this.isCamera = isCamera;
        visionThread = new VisionThread(this, camera);
        telemetryDump.registerTelemeter(visionThread);

        debugThread = new DebugThread(this, DEBUG);
//        telemetryDump.registerTelemeter(debugThread);

        this.start();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this(hardwareMap, telemetry, linearOpMode, true);
    }

    public void start() {
        Thread[] threads;
        if (this.isCamera) {
            threads = new Thread[]{
                    new Thread(sensorThread),
                    new Thread(moduleThread),
                    new Thread(visionThread),
                    new Thread(debugThread)
            };
        } else {
            threads = new Thread[]{
                    new Thread(sensorThread),
                    new Thread(moduleThread),
                    new Thread(debugThread)
            };
        }

        for (Thread thread : threads) {
            thread.start();
        }
    }



    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isAuto() {
        return linearOpMode.getClass().isAnnotationPresent(Autonomous.class);
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

    @Override
    public Pose getPose() {
        return sensorThread.getPose();
    }

    @Override
    public Pose getVelocity() {
        return sensorThread.getVelocity();
    }

    public void resetPose(Pose pose) {
        sensorThread.resetPose(pose);
    }

    public void followPath(PurePursuit path) {
        path.reset();

        telemetryDump.registerTelemeter(path);
        while (isOpModeActive() && path.update(this, drivetrain));
        telemetryDump.removeTelemeter(path);
    }
}
