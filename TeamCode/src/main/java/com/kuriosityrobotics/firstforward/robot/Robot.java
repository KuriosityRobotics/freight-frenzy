package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.debug.DebugThread;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
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
    private static boolean blue = false;
    private static boolean carousel = false;

    private final SensorThread sensorThread;
    private final ModuleThread moduleThread;
    private final VisionThread visionThread;
    private final DebugThread debugThread;

    private final Drivetrain drivetrain;
    private final IntakeModule intakeModule;
    private final OuttakeModule outtakeModule;
    private final CarouselModule carouselModule;
    private final TelemetryDump telemetryDump;

    private final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;
    private final LynxModule controlHub;
    private final LynxModule expansionHub;
    private final WebcamName camera;
    private final boolean useCamera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode, boolean useCamera) {
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        this.camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetryDump = new TelemetryDump(telemetry);

        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }

        ActionExecutor.reset();

        // init sensorThread up here since drivetrain depends on it
        sensorThread = new SensorThread(this);
        telemetryDump.registerTelemeter(sensorThread);

        // modules
        drivetrain = new Drivetrain(sensorThread.getOdometry(), hardwareMap);
        telemetryDump.registerTelemeter(drivetrain);

        outtakeModule = new OuttakeModule(this, hardwareMap);
        telemetryDump.registerTelemeter(outtakeModule);

        intakeModule = new IntakeModule(hardwareMap, outtakeModule);
        telemetryDump.registerTelemeter(intakeModule);

        carouselModule = new CarouselModule(hardwareMap);
        telemetryDump.registerTelemeter(carouselModule);

        LEDModule ledModule = new LEDModule(this);
        telemetryDump.registerTelemeter(ledModule);

        // threads
        moduleThread = new ModuleThread(this,
                drivetrain,
                intakeModule,
                outtakeModule,
                carouselModule,
                ledModule
        );

        this.useCamera = useCamera;
        visionThread = new VisionThread(this, camera);

        debugThread = new DebugThread(this, DEBUG);
//        telemetryDump.registerTelemeter(debugThread);

        this.start();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this(hardwareMap, telemetry, linearOpMode, true);
    }

    public static void assertThat(boolean condition, String message) {
        if (!condition)
            if (DEBUG)
                throw new AssertionError(message);
            else
                new AssertionError(message).printStackTrace();
    }

    public static void assertThat(boolean condition) {
        assertThat(condition, "Assertion failed");
    }

    public static boolean isBlue() {
        return blue;
    }

    public static void setBlue(boolean blue) {
        Robot.blue = blue;
    }

    public static boolean isCarousel() {
        return carousel;
    }

    public static void setCarousel(boolean carousel) {
        Robot.carousel = carousel;
    }

    public void start() {
        Thread[] threads;
        if (this.useCamera) {
            threads = new Thread[]{
                    new Thread(sensorThread, "SensorThread"),
                    new Thread(moduleThread, "ModuleThread"),
                    new Thread(visionThread, "VisionThread"),
                    new Thread(debugThread, "DebugThread")
            };
        } else {
            threads = new Thread[]{
                    new Thread(sensorThread, "SensorThread"),
                    new Thread(moduleThread, "ModuleThread"),
                    new Thread(debugThread, "DebugThread")
            };
        }

        for (Thread thread : threads) {
            thread.start();
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isTeleOp() {
        return !linearOpMode.getClass().isAnnotationPresent(Autonomous.class);
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

    public SensorThread getSensorThread() {
        return sensorThread;
    }

    public VisionThread getVisionThread() {
        return visionThread;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public IntakeModule getIntakeModule() {
        return intakeModule;
    }

    public OuttakeModule getOuttakeModule() {
        return outtakeModule;
    }

    public CarouselModule getCarouselModule() {
        return carouselModule;
    }

    public TelemetryDump getTelemetryDump() {
        return telemetryDump;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }

    public WebcamName isUseCamera() {
        return camera;
    }
}
