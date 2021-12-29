package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    //time constants
    private static final long HOPPER_EXTEND_TIME = 500;
    private static final long HOPPER_ROTATE_TIME = 800;
    private static final long HOPPER_DUMP_TIME = 600;
    private static final long SLIDE_RAISE_TIME = 0;

    //constants
    private static final double LINKAGE_EXTENDED = 0.9;
    private static final double LINKAGE_RETRACTED = 0.087;
    private static final double HOPPER_PIVOT_IN = 0.906181;
    private static final double HOPPER_RESTING_POSITION = .692386;
    public static double pivotHeading;
    public static boolean skipRotate = false;

    public enum HopperDumpPosition {
        DUMP_OUTWARDS(.0),
        DUMP_INWARDS(.9887);

        private final double position;

        HopperDumpPosition(double position) {
            this.position = position;
        }
    }

    public enum VerticalSlideLevel {
        TOP(-885),
        MID(-466),
        DOWN(-2);

        private final int position;

        VerticalSlideLevel(int position) {
            this.position = position;
        }
    }

    private final Robot robot;
    private static double robotHeading;

    // states
    public static VerticalSlideLevel slideLevel = VerticalSlideLevel.DOWN;
    private OuttakeState outtakeState = OuttakeState.IDLE;

    //servos
    private static Servo linkage;
    private static Servo pivot;
    private static Servo hopper;

    //motors
    private static DcMotor slide;

    // helpers
    private static HopperDumpPosition dumpMode = HopperDumpPosition.DUMP_OUTWARDS;
    private final boolean isOn = true;
    private boolean isHopperOccupied = false;

    public enum OuttakeState {
        //        SLIDES_UP(SLIDE_RAISE_TIME, () -> {
//            slide.setPower(1);
//            slide.setTargetPosition(slideLevel.position);
//        }),
        LINKAGE_OUT(HOPPER_EXTEND_TIME, () -> linkage.setPosition(LINKAGE_EXTENDED)),
        DUMP(HOPPER_DUMP_TIME, () -> hopper.setPosition(dumpMode.position)),
        HOPPER_RESTING(1, () -> hopper.setPosition(HOPPER_RESTING_POSITION)),
        HOPPER_RETURNING(HOPPER_ROTATE_TIME, () -> pivot.setPosition(HOPPER_PIVOT_IN)),
        LINKAGE_IN(HOPPER_EXTEND_TIME, () -> {
            linkage.setPosition(LINKAGE_RETRACTED);
        }),
        SLIDES_DOWN(SLIDE_RAISE_TIME, () -> {
            slideLevel = VerticalSlideLevel.DOWN;
        }),
        IDLE(0, () -> {}),
        IDLE_ROTATE(0, () -> {});

        public final double completionTime;
        private final Runnable onStart;
        private Long currentStateStartTimeMillis;

        OuttakeState(double completionTime, Runnable onStart) {
            this.completionTime = completionTime;
            this.onStart = onStart;
        }

//        public static OuttakeState startRaiseSequence() {
//            SLIDES_UP.currentStateStartTimeMillis = SystemClock.elapsedRealtime();
//            SLIDES_UP.onStart.run();
//            return SLIDES_UP;
//        }

        public static OuttakeState startDumpSequence(HopperDumpPosition position) {
            OuttakeModule.dumpMode = position;
            LINKAGE_OUT.currentStateStartTimeMillis = SystemClock.elapsedRealtime();
            LINKAGE_OUT.onStart.run();
            return LINKAGE_OUT;
        }

        public boolean shouldTransition() {
            return SystemClock.elapsedRealtime() - currentStateStartTimeMillis > completionTime;
        }

        public void executeTransition() {
            this.currentStateStartTimeMillis = SystemClock.elapsedRealtime();
            this.onStart.run();
        }
    }


    public OuttakeModule(Robot robot) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);

        linkage = robot.getServo("outtakeLinkage");
        pivot = robot.getServo("pivot");
        hopper = robot.getServo("hopper");
        slide = robot.getDcMotor("lift");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        hopper.setPosition(HOPPER_RESTING_POSITION);
        pivot.setPosition(HOPPER_PIVOT_IN);
        linkage.setPosition(LINKAGE_RETRACTED);

        slideLevel = VerticalSlideLevel.DOWN;
        outtakeState = OuttakeState.IDLE;
    }

    public void dump(HopperDumpPosition dumpMode) {
        if (this.outtakeState == OuttakeState.IDLE) {
            this.outtakeState = OuttakeState.startDumpSequence(dumpMode);
        }
    }

    public void hopperOccupied() {
        isHopperOccupied = true;
    }

    public boolean readyForIntake() {
        return slide.getCurrentPosition() > -50
                && Math.abs(pivot.getPosition() - HOPPER_PIVOT_IN) < 0.1;
    }

    public void update() {
        double robotHeading = robot.sensorThread.getPose().heading;
        if (this.outtakeState != OuttakeState.IDLE && this.outtakeState.shouldTransition()) {
            switch (this.outtakeState) {
                case LINKAGE_OUT:
                    this.outtakeState = OuttakeState.IDLE_ROTATE;
                    break;
                case IDLE_ROTATE:
                    if (skipRotate){
                        this.outtakeState = OuttakeState.DUMP;
                        break;
                    }
                    //turning right increases angle for some reason
                    pivot.setPosition(radToServoPos(pivotHeading + robotHeading));
                    this.outtakeState = OuttakeState.IDLE_ROTATE;
                    break;
                case DUMP:
                    this.outtakeState = OuttakeState.HOPPER_RESTING;
                    this.isHopperOccupied = false;
                    break;
                case HOPPER_RESTING:
                    this.outtakeState = OuttakeState.HOPPER_RETURNING;
                    break;
                case HOPPER_RETURNING:
                    this.outtakeState = OuttakeState.LINKAGE_IN;
                    break;
                case LINKAGE_IN:
                    this.outtakeState = OuttakeState.SLIDES_DOWN;
                    break;
                case SLIDES_DOWN:
                    this.outtakeState = OuttakeState.IDLE;
//                    slide.setPower(0);
                    break;
            }

            this.outtakeState.executeTransition();
        }

        if (slide.getTargetPosition() >= -15) {
            slide.setPower(0);
        } else {
            slide.setPower(1);
        }

        if (robot.intakeModule.isIntakeRetracting()) {
            slide.setTargetPosition(VerticalSlideLevel.DOWN.position);
        } else {
            slide.setTargetPosition(slideLevel.position);
        }
    }

    public OuttakeState getOuttakeState() {
        return this.outtakeState;
    }

    public double radToServoPos(double rad){
        //0 is .906181;  -90 is .5738778;  -180 is .23151
        return .00374817222 * (rad * 180/Math.PI) + 0.906181;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "OuttakeModule";
    }

    @Override
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>() {{
            add("State:  " + outtakeState.toString());
            add("slideLevel: " + slideLevel.name());
            add("--");
            add("slide target:  " + slide.getTargetPosition());
            add("current slide:  " + slide.getCurrentPosition());
            add("hopper occupied:  " + isHopperOccupied);
        }};
        return data;
    }
}
