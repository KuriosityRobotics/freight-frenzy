package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.EXTEND;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.RETURN_TURRET;
import static java.lang.Math.abs;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    //time constants
    private static final long EXTEND_TIME = 1000;
    private static final long DUMP_TIME = 1000;
    private static final long TURRET_TIME = 750; // if the turret isn't already straight

    // intake clamp pos:  .82761
    // clamp clamp pos:  .5277
    // clamp release pos:  .9025
    private static final double CLAMP_INTAKE = .82761,
            CLAMP_CLAMP = .5277,
            CLAMP_RELEASE = .9025;
    // linkage in = .77067
    // linkage extended:  .167304
    private static final double LINKAGE_IN = .77067,
            LINKAGE_EXTENDED = .167304;
    // pivot down:  .993
    // pivot in:  .0060539
    private static final double PIVOT_OUT = .993,
            PIVOT_IN = .0060539;

    // turret @ -pi/2:  .78988
    // turret @ pi:  .4896
    // turret @ pi/2 .1906
    // from the perspective of looking out from the back of the robot
    public enum TurretPosition {
        STRAIGHT(.4896),
        RIGHT(.1906),
        LEFT(.78988);

        private final double position;

        TurretPosition(double position) {
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

    public enum OuttakeState {
        RAISE(0),
        EXTEND(EXTEND_TIME),
        DUMP(DUMP_TIME),
        RETURN_TURRET(0),
        COLLAPSE(0);

        public final long completionTime;

        OuttakeState(long completionTime) {
            this.completionTime = completionTime;
        }
    }

    private boolean phaseComplete() {
        long currentTime = System.currentTimeMillis();
        boolean timerComplete = currentTime > transitionTime + currentState.completionTime;
        boolean slidesAtTarget = abs(slide.getCurrentPosition() - slide.getTargetPosition()) < 50;

        if (currentState == COLLAPSE) {
            return timerComplete && slidesAtTarget;
        } else if (currentState == EXTEND) {
            if (targetTurret == TurretPosition.STRAIGHT) {
                return timerComplete && slidesAtTarget;
            } else {
                return currentTime >= (transitionTime + currentState.completionTime + TURRET_TIME) && slidesAtTarget;
            }
        } else if (currentState == RETURN_TURRET) {
            if (targetTurret == TurretPosition.STRAIGHT)
                return currentTime >= (transitionTime + currentState.completionTime + TURRET_TIME);
        }
        return timerComplete;
    }

    private final Robot robot;

    // states
    public VerticalSlideLevel targetSlideLevel;
    public OuttakeState targetState;
    public TurretPosition targetTurret;

    private OuttakeState currentState;

    //servos
    private final Servo linkage;
    private final Servo pivot;
    private final Servo clamp;
    private final Servo turret;

    //motors
    private final DcMotor slide;

    // helpers
    private long transitionTime;

    public OuttakeModule(Robot robot) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);

        linkage = robot.getServo("outtakeLinkage");
        pivot = robot.getServo("outtakePivot");
        clamp = robot.getServo("outtakeClamp");
        turret = robot.getServo("turret");

        slide = robot.getDcMotor("lift");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp.setPosition(CLAMP_INTAKE);
        pivot.setPosition(PIVOT_IN);
        linkage.setPosition(LINKAGE_IN);

        targetSlideLevel = VerticalSlideLevel.DOWN;
        currentState = OuttakeState.COLLAPSE;
    }

    public void skipToCollapse() {
        this.targetState = COLLAPSE;
        this.currentState = RETURN_TURRET;
    }

    String lastRan = "";

    public void update() {
        if (phaseComplete() && currentState != targetState) {
            lastRan = currentState.name();

            currentState = OuttakeState.values()[currentState.ordinal() + 1];

            switch (this.currentState) {
                case RAISE:
                    clamp.setPosition(CLAMP_CLAMP);
                    slide.setTargetPosition(targetSlideLevel.position);
                    break;
                case EXTEND:
                    linkage.setPosition(LINKAGE_EXTENDED);
                    pivot.setPosition(PIVOT_OUT);
                    break;
                case DUMP:
                    clamp.setPosition(CLAMP_RELEASE);
                    break;
                case RETURN_TURRET:
                    turret.setPosition(TurretPosition.STRAIGHT.position);
                    break;
                case COLLAPSE:
                    clamp.setPosition(CLAMP_INTAKE);
                    pivot.setPosition(PIVOT_IN);
                    linkage.setPosition(LINKAGE_IN);
                    slide.setTargetPosition(VerticalSlideLevel.DOWN.getPosition());
                    break;
            }

            transitionTime = System.currentTimeMillis();
        }

        if (slide.getTargetPosition() >= -15) {
            slide.setPower(0);
        } else {
            slide.setPower(1);
        }

        if (currentState == EXTEND && phaseComplete()) {
            slide.setTargetPosition(targetSlideLevel.position);
            pivot.setPosition(targetTurret.position);
        }
    }

    public boolean atTargetState() {
        return atState(targetState);
    }

    public boolean atState(OuttakeState state) {
        return currentState == state && phaseComplete();
    }

    public boolean collapsed() {
        return atState(COLLAPSE);
    }

    public OuttakeState getCurrentState() {
        return this.currentState;
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
            add("Target State: " + targetState.toString());
            add("State:  " + currentState.toString());
//            add("last:  " + lastRan);
            add("slideLevel: " + targetSlideLevel.name());
            add("Turret: " + targetTurret.name());
//            add("--");
//            add("slide target:  " + slide.getTargetPosition());
//            add("current slide:  " + slide.getCurrentPosition());
        }};
    }

}
