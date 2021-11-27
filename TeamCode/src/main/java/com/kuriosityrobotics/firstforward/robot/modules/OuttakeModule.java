package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    //constants
    private static final double HOPPER_EXTENDED = 0.9;
    private static final double HOPPER_RETRACTED = 0.087;
    private static final double HOPPER_PIVOT_IN = 0.906181;
    private static final double HOPPER_HOLD = .7;
    //TODO determine actual marking

    public enum HopperDumpPosition {
        DUMP_OUTWARDS(.1593),
        DUMP_INWARDS(.9887);

        private final double position;

        HopperDumpPosition(double position) {
            this.position = position;
        }
    }

    public enum VerticalSlideLevel {
        TOP(-410),
        MID(-205),
        DOWN(0);

        private final int position;

        VerticalSlideLevel(int position) {
            this.position = position;
        }
    }

    //ACTIONS
    //time constants
    //TODO determine time
    private static final long HOPPER_EXTEND_TIME = 500;
    private static final long HOPPER_ROTATE_TIME = 950;
    private static final long HOPPER_DUMP_TIME = 600;
    private static final long SLIDE_RAISE_TIME = 350;
    private final boolean isOn = true;
    private boolean isHopperOccupied = false;

    private final Robot robot;

    //servos
    private static Servo linkage;
    private static Servo pivot;
    private static Servo hopper;
    //motors
    private static DcMotor slide;
    public static VerticalSlideLevel slideLevel = VerticalSlideLevel.DOWN;
    public static HopperDumpPosition dumpMode = HopperDumpPosition.DUMP_OUTWARDS;

    private OuttakeState outtakeState = OuttakeState.IDLE;

    enum OuttakeState {
//        SLIDES_UP(SLIDE_RAISE_TIME, () -> {
//            slide.setPower(1);
//            slide.setTargetPosition(slideLevel.position);
//        }),
        LINKAGE_OUT(HOPPER_EXTEND_TIME, () -> linkage.setPosition(HOPPER_EXTENDED)),
        DUMP(HOPPER_DUMP_TIME, () -> hopper.setPosition(dumpMode.position)),
        HOLD(1, () -> hopper.setPosition(HOPPER_HOLD)),
        LINKAGE_IN(HOPPER_EXTEND_TIME, () -> {
            pivot.setPosition(HOPPER_PIVOT_IN);
            linkage.setPosition(HOPPER_RETRACTED);
        }),
        SLIDES_DOWN(SLIDE_RAISE_TIME, () -> {
            slide.setTargetPosition(VerticalSlideLevel.DOWN.position);
        }),
        IDLE(0, () -> {
        });

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

        linkage = robot.getServo("linkage_servo");
        pivot = robot.getServo("pivot_servo");
        hopper = robot.getServo("hopper_pivot_servo");
        slide = robot.getDcMotor("linear_slide_motor");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        hopper.setPosition(HOPPER_HOLD);
        pivot.setPosition(HOPPER_PIVOT_IN);
        linkage.setPosition(HOPPER_RETRACTED);
    }

    public void setLevel(VerticalSlideLevel slideLevel) {
        OuttakeModule.slideLevel = slideLevel;
    }

    public void dump(HopperDumpPosition dumpMode) {
        if (this.outtakeState == OuttakeState.IDLE) {
            this.outtakeState = OuttakeState.startDumpSequence(dumpMode);
        }
    }

    public void hopperOccupied() {
        isHopperOccupied = true;
    }

    public void update() {
        if (this.isHopperOccupied) {
            slide.setPower(1);
            slide.setTargetPosition(slideLevel.position);
        }


        if (this.outtakeState != OuttakeState.IDLE && this.outtakeState.shouldTransition()) {
            switch (this.outtakeState) {
//                case SLIDES_UP:
//                    this.outtakeState = OuttakeState.IDLE;
//                    break;
                case LINKAGE_OUT:
                    this.outtakeState = OuttakeState.DUMP;
                    break;
                case DUMP:
                    this.outtakeState = OuttakeState.HOLD;
                    this.isHopperOccupied = false;
                    break;
                case HOLD:
                    this.outtakeState = OuttakeState.LINKAGE_IN;
                    break;
                case LINKAGE_IN:
                    this.outtakeState = OuttakeState.SLIDES_DOWN;
                    break;
                case SLIDES_DOWN:
                    this.outtakeState = OuttakeState.IDLE;
                    slide.setPower(0);
                    break;
            }

            this.outtakeState.executeTransition();
        }
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
            add("slide target:  " + slide.getTargetPosition());
            add("current slide:  " + slide.getCurrentPosition());
        }};
        return data;
    }
}
