package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.IDLE;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.LINKAGE_OUT;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.SLIDES_DOWN;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.SLIDES_UP;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.WAIT_FOR_COMMAND2;
import static java.lang.Math.abs;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    //intake clamp pos:  .82761
    // clamp clamp pos:  .5277
    //  clamp release pos:  .9025

    private static final double CLAMP_INTAKE = .82761,
            CLAMP_CLAMP = .5277,
            CLAMP_RELEASE = .9025;
    // turret @ -pi/2:  .78988
    // turret @ pi:  .4896
    // turret @ pi/2 .1906
    private static final double TURRET_270 = .78988,
            TURRET_180 = .4896,
            TURRET_90 = .1906;
    // pivot down:  .993
    // pivot in:  .0060539
    private static final double PIVOT_OUT = .993,
            PIVOT_IN = .0060539;
    // linkage in = .77067
    // linkage extended:  .167304
    private static final double LINKAGE_IN = .77067,
            LINKAGE_EXTENDED = .167304;



    //time constants
    private static final long HOPPER_EXTEND_TIME = 800;
    private static final long HOPPER_PIVOT_TIME = 600;
    private static final long HOPPER_DUMP_TIME = 500;
    private static final long SLIDE_RAISE_TIME = 800;

    private boolean stateJustSetManually;

    private double turretPosition = TURRET_180;

    // DUMPER FLAT: 0.350542
    // inwards: 0
    // outwards: 0.82091
    // intake: 0.53552

    public enum HopperDumpPosition {
        DUMP_OUTWARDS(0.82091),
        DUMP_INWARDS(0.0);

        private final double position;

        HopperDumpPosition(double position) {
            this.position = position;
        }
    }

    public enum VerticalSlideLevel {
        TOP_TOP(-1535),
        TOP(-1275),
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

    public void setOuttakeState(OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    private OuttakeState outtakeState;

    //servos
    private final Servo linkage;
    private final Servo pivot;
    private final Servo clamp;

    private final Object lock = new Object();

    //motors
    private final DcMotor slide;

    // helpers
    private HopperDumpPosition dumpMode = HopperDumpPosition.DUMP_OUTWARDS;
    private boolean isHopperOccupied = false;
    private boolean extendSlides = false;

    public enum OuttakeState {
        CLAMP_CLAMP(0),
        SLIDES_UP(0),
        LINKAGE_OUT(0),
        PIVOT_OUT(HOPPER_PIVOT_TIME),
        WAIT_FOR_COMMAND2(0),
        CLAMP_RELEASE(100),
        CLAMP_INTAKE(HOPPER_DUMP_TIME),
        PIVOT_IN(0),
        LINKAGE_IN(0),
        SLIDES_DOWN(SLIDE_RAISE_TIME),
        IDLE(0);

        public final long completionTime;

        OuttakeState(long completionTime) {
            this.completionTime = completionTime;
        }
    }

    public void pivotRight() {
        turretPosition = TURRET_90;
        tryMoveLinkageOut();
    }

    public void pivotStraight() {
        turretPosition = TURRET_180;
        tryMoveLinkageOut();
    }

    public void pivot270() {
        turretPosition = TURRET_270;
        tryMoveLinkageOut();
    }

    public void pivotIn() {
        turretPosition = TURRET_180;
        tryMoveLinkageOut();
    }

    private void tryMoveLinkageOut() {
        if (outtakeState == WAIT_FOR_COMMAND2 || outtakeState == IDLE) {
            outtakeState = LINKAGE_OUT;
            stateJustSetManually = true;
        }
        this.startPhaseTimer(outtakeState.completionTime);
    }


    public OuttakeModule(Robot robot) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);

        linkage = robot.getServo("outtakeLinkage");
        pivot = robot.getServo("outtakePivot");
        clamp = robot.getServo("outtakeClamp");
        slide = robot.getDcMotor("lift");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        clamp.setPosition(CLAMP_INTAKE);
        pivot.setPosition(PIVOT_IN);
        linkage.setPosition(LINKAGE_IN);

        slideLevel = VerticalSlideLevel.DOWN;
        outtakeState = OuttakeState.IDLE;
    }

    public void dump(HopperDumpPosition dumpMode) {
        synchronized (lock) {
            if (this.outtakeState == WAIT_FOR_COMMAND2) {
                this.dumpMode = dumpMode;
                this.startPhaseTimer(outtakeState.completionTime);
                stateJustSetManually = true;
                this.outtakeState = OuttakeState.CLAMP_RELEASE;
            }
        }
    }

    public void hopperOccupied() {
        isHopperOccupied = true;
    }

    public boolean readyForIntake() {
        return slide.getCurrentPosition() > -50
                && abs(pivot.getPosition() - PIVOT_IN) < 0.1;
    }

    private long phaseCompletionTime;

    private boolean phaseComplete() {
        if (outtakeState == SLIDES_UP || outtakeState == SLIDES_DOWN)
            return abs(slide.getCurrentPosition() - slide.getTargetPosition()) < 50;
        else if (outtakeState == OuttakeState.PIVOT_IN) {
            if (turretPosition >= TURRET_270)
                return System.currentTimeMillis() >= (phaseCompletionTime + 500);
        }

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
        if (outtakeState != IDLE && outtakeState != WAIT_FOR_COMMAND2 && outtakeState != WAIT_FOR_COMMAND2) {
            outtakeState = OuttakeState.values()[outtakeState.ordinal() + 1];
        }
    }

    String lastRan = "";

    public void update() {
        synchronized (lock) {
            if (stateJustSetManually || phaseComplete()) {
                if (!stateJustSetManually) advanceState();
                else stateJustSetManually = false;

                lastRan = outtakeState.name();

                switch (this.outtakeState) {
                    case CLAMP_CLAMP:
                        clamp.setPosition(CLAMP_CLAMP);
                        break;
                    case SLIDES_UP:
                        slideLevel = VerticalSlideLevel.TOP;
                        break;
                    case LINKAGE_OUT:
                        linkage.setPosition(LINKAGE_EXTENDED);
                        break;
                    case PIVOT_OUT:
                        pivot.setPosition(PIVOT_OUT);
                        break;
                    case WAIT_FOR_COMMAND2:
                        break;
                    case CLAMP_RELEASE:
                        clamp.setPosition(CLAMP_RELEASE);
                        break;
                    case CLAMP_INTAKE:
                        clamp.setPosition(CLAMP_INTAKE);
                        break;
                    case PIVOT_IN:
                        pivot.setPosition(PIVOT_IN);
                        break;
                    case LINKAGE_IN:
                        linkage.setPosition(LINKAGE_IN);
                        break;
                    case SLIDES_DOWN:
                        slideLevel = VerticalSlideLevel.DOWN;
                        break;
                    case IDLE:
                        break;
                }

                this.startPhaseTimer(outtakeState.completionTime);
            }

            if (extendSlides) {
                linkage.setPosition((turretPosition == TURRET_90 || turretPosition == TURRET_270) ? 0.55 : LINKAGE_EXTENDED);
            } else {
                linkage.setPosition(LINKAGE_IN);
            }

            if (slide.getTargetPosition() >= -15) {
                slide.setPower(0);
            } else {
                slide.setPower(1);
            }

            if ((outtakeState != IDLE)
                    && outtakeState.ordinal() > LINKAGE_OUT.ordinal()
                    && outtakeState.ordinal() < OuttakeState.PIVOT_IN.ordinal())
                pivot.setPosition(turretPosition);
            else
                pivot.setPosition(PIVOT_IN);


            slide.setTargetPosition(slideLevel.position);
        }
    }

    public void raise() {
        if (this.outtakeState == IDLE) {
            this.outtakeState = SLIDES_UP;
            stateJustSetManually = true;
            this.startPhaseTimer(outtakeState.completionTime);
        }
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

    {
        var a = "";
    }

    @Override
    public Iterable<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("State:  " + outtakeState.toString());
//            add("last:  " + lastRan);
            add("slideLevel: " + slideLevel.name());
//            add("--");
//            add("slide target:  " + slide.getTargetPosition());
//            add("current slide:  " + slide.getCurrentPosition());
//            add("hopper occupied:  " + isHopperOccupied);
//            add("timer:  " + (phaseCompletionTime - System.currentTimeMillis()));
        }};
    }

    public VerticalSlideLevel getSlideLevel() {
        return slideLevel;
    }

    public void setSlideLevel(VerticalSlideLevel slideLevel) {
        if (slideLevel == VerticalSlideLevel.DOWN) {
            outtakeState = OuttakeState.PIVOT_IN;
            this.startPhaseTimer(outtakeState.completionTime);
            stateJustSetManually = true;
        } else
            this.slideLevel = slideLevel;

        if (this.slideLevel == VerticalSlideLevel.DOWN) {
            clamp.setPosition(CLAMP_INTAKE);
        }
    }
}
