package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.DUMP;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.IDLE;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.WAIT_FOR_COMMAND;

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
        TOP(-1012),
        MID(-575),
        DOWN(-2);

        private final int position;

        VerticalSlideLevel(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    private final Robot robot;

    // states
    private VerticalSlideLevel slideLevel;
    private OuttakeState outtakeState;

    //servos
    private final Servo linkage;
    private final Servo pivot;
    private final Servo hopper;

    //motors
    private final DcMotor slide;

    // helpers
    private final HopperDumpPosition dumpMode = HopperDumpPosition.DUMP_OUTWARDS;
    private boolean isHopperOccupied = false;

    public enum OuttakeState {
        LINKAGE_OUT(HOPPER_EXTEND_TIME),
        WAIT_FOR_COMMAND(0),
        DUMP(HOPPER_DUMP_TIME),
        HOPPER_RESTING(1),
        HOPPER_RETURNING(HOPPER_ROTATE_TIME),
        LINKAGE_IN(HOPPER_EXTEND_TIME),
        SLIDES_DOWN(SLIDE_RAISE_TIME),
        IDLE(0);

        public final long completionTime;

        OuttakeState(long completionTime) {
            this.completionTime = completionTime;
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
        if (this.outtakeState == IDLE) {
            this.outtakeState = DUMP;
        }
    }

    public void hopperOccupied() {
        isHopperOccupied = true;
    }

    public boolean readyForIntake() {
        return slide.getCurrentPosition() > -50
                && Math.abs(pivot.getPosition() - HOPPER_PIVOT_IN) < 0.1;
    }

    private long phaseCompletionTime;

    private boolean phaseComplete() {
        return System.currentTimeMillis() >= phaseCompletionTime;
    }

    private void startPhaseTimer(long millis) {
        phaseCompletionTime = System.currentTimeMillis() + millis;
    }

    /**
     * advances to the next state.  if currentState is IDLE or WAIT_FOR_COMMAND,
     * function is identity.  this is because these states must be manually
     * exited.
     */
    private void advanceState() {
        if (outtakeState != IDLE && outtakeState != WAIT_FOR_COMMAND) {
            outtakeState = OuttakeState.values()[outtakeState.ordinal() + 1];
        }
    }

    public void update() {
        if (phaseComplete()) {
            advanceState();
            switch (this.outtakeState) {
                case LINKAGE_OUT:
                    linkage.setPosition(LINKAGE_EXTENDED);
                    break;
                case DUMP:
                    hopper.setPosition(dumpMode.position);
                    this.isHopperOccupied = false;
                    break;
                case HOPPER_RESTING:
                    hopper.setPosition(HOPPER_RESTING_POSITION);
                    break;
                case HOPPER_RETURNING:
                    pivot.setPosition(HOPPER_PIVOT_IN);
                    break;
                case LINKAGE_IN:
                    linkage.setPosition(LINKAGE_RETRACTED);
                    break;
                case SLIDES_DOWN:
                    slideLevel = VerticalSlideLevel.DOWN;
                    break;
            }

            this.startPhaseTimer(outtakeState.completionTime);
        }

        if (slide.getTargetPosition() >= -15) {
            slide.setPower(0);
        } else {
            slide.setPower(1);
        }

        slide.setTargetPosition(slideLevel.position);
    }

    public OuttakeState getOuttakeState() {
        return this.outtakeState;
    }

    public double radToServoPos(double rad) {
        //0 is .906181;  -90 is .5738778;  -180 is .23151
        return .00374817222 * (rad * 180 / Math.PI) + 0.906181;
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "OuttakeModule";
    }

    @Override
    public Iterable<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("State:  " + outtakeState.toString());
            add("slideLevel: " + slideLevel.name());
            add("--");
            add("slide target:  " + slide.getTargetPosition());
            add("current slide:  " + slide.getCurrentPosition());
            add("hopper occupied:  " + isHopperOccupied);
        }};
    }

    public VerticalSlideLevel getSlideLevel() {
        return slideLevel;
    }

    public void setSlideLevel(VerticalSlideLevel slideLevel) {
        slide.setTargetPosition(slideLevel.getPosition());
    }
}
