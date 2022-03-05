package com.kuriosityrobotics.firstforward.robot.modules.outtake;

import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.DUMP;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.EXTEND;
import static java.lang.Math.abs;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    LocationProvider locationProvider;

    //time constants
    private static final long EXTEND_TIME = 300;
    private static final long DUMP_TIME = 500;
    private static final long TURRET_TIME = 150; // if the turret isn't already straight

    private static final double CLAMP_INTAKE = .818367,
            CLAMP_CLAMP = .75,
            CLAMP_RELEASE = 0.890738;
    private static final double LINKAGE_IN = .140102,
            LINKAGE_EXTENDED = .8777921, LINKAGE_PARTIAL_EXTEND = 0.45;
    // pivot down:  .993
    // pivot in:  .0060539
    private static final double PIVOT_OUT = .993,
            PIVOT_UP = 0.5,
            PIVOT_IN = .0060539;

    private static double ALLIANCE_HUB_TURRET_POSITION = 0.5;

    // from the perspective of looking out from the back of the robot
    public enum TurretPosition {
        STRAIGHT(.482746),
        RIGHT(.78988),
        LEFT(.186781),
        ALLIANCEHUBLOCK(0.5);

        private final double position;

        TurretPosition(double position) {
            this.position = position;
        }
    }

    public enum VerticalSlideLevel {
        TOP_TOP(-1150),
        TOP(-900),
        MID(-350),
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
        TURRET_IN(0),
        RETRACT(EXTEND_TIME),
        COLLAPSE(0);

        public final long completionTime;

        OuttakeState(long completionTime) {
            this.completionTime = completionTime;
        }
    }

    private boolean phaseComplete() {
        long currentTime = System.currentTimeMillis();
        boolean timerComplete = currentTime >= transitionTime + currentState.completionTime;
        boolean turretTimerComplete = currentTime >= transitionTime + currentState.completionTime + TURRET_TIME;
        boolean slidesAtTarget = abs(slide.getCurrentPosition() - slide.getTargetPosition()) < 50;

        switch (currentState) {
            case RAISE:
                if (targetSlideLevel == VerticalSlideLevel.TOP || targetSlideLevel == VerticalSlideLevel.TOP_TOP) {
                    return slidesAtTarget;
                } else {
                    return true;
                }
            case COLLAPSE:
                return timerComplete && slidesAtTarget;
            case EXTEND:
                if (targetTurret == TurretPosition.STRAIGHT) {
                    return timerComplete && slidesAtTarget;
                } else {
                    return turretTimerComplete && slidesAtTarget;
                }
            case RETRACT:
                if (targetSlideLevel == VerticalSlideLevel.TOP || targetSlideLevel == VerticalSlideLevel.TOP_TOP) {
                    return true;
                } else {
                    return timerComplete;
                }
            case TURRET_IN:
                if (targetTurret == TurretPosition.STRAIGHT)
                    return true;
                else
                    return turretTimerComplete;
            default:
                return timerComplete;
        }
    }

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
    private final DcMotorEx slide;

    // helpers
    private long transitionTime;

    public OuttakeModule(LocationProvider locationProvider, HardwareMap hardwareMap) {
        this.locationProvider = locationProvider;

//        linkage = hardwareMap.servo.get("outtakeLinkage");
//        pivot = hardwareMap.servo.get("outtakePivot");
//        clamp = hardwareMap.servo.get("outtakeClamp");
//        turret = hardwareMap.servo.get("outtakeTurret");

        //FIXME: change this back when outtake doesnt try to tear itself to shreds
        linkage = hardwareMap.servo.get("nothingServo");
        pivot = hardwareMap.servo.get("nothingServo");
        clamp = hardwareMap.servo.get("nothingServo");
        turret = hardwareMap.servo.get("nothingServo");

        slide = (DcMotorEx) hardwareMap.dcMotor.get("lift");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 20));

        clamp.setPosition(CLAMP_INTAKE);
        pivot.setPosition(PIVOT_IN);
        linkage.setPosition(LINKAGE_IN);

        this.targetSlideLevel = VerticalSlideLevel.DOWN;
        this.targetState = COLLAPSE;
        this.targetTurret = TurretPosition.STRAIGHT;

        this.currentState = COLLAPSE;
    }

    public void skipToCollapse() {
        transitionTime = 0;
        this.targetState = COLLAPSE;
        this.currentState = DUMP;
    }

    public void defaultFullExtend() {
        this.targetTurret = TurretPosition.STRAIGHT;
        this.targetSlideLevel = VerticalSlideLevel.TOP;
        this.targetState = EXTEND;
    }

    String lastRan = "";

    public void update() {
        if (phaseComplete() && currentState != targetState) {
            lastRan = currentState.name();

            currentState = OuttakeState.values()[currentState.ordinal() + 1 >= OuttakeState.values().length ? 0 : currentState.ordinal() + 1];

            switch (this.currentState) {
                case RAISE:
                    linkage.setPosition(LINKAGE_PARTIAL_EXTEND);
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
                case RETRACT:
                    linkage.setPosition(LINKAGE_IN);
                    break;
                case TURRET_IN:
                    pivot.setPosition(PIVOT_UP);
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

        if (currentState == EXTEND && atTargetState()) {
            turret.setPosition(targetTurret.position);
        }

        // if current position is higher than the target
        if (currentState == COLLAPSE) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0);
        } else {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition(targetSlideLevel.position);
            slide.setPower(1);
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
//            add("current slide:  " + slide.getCurrentPosition());
        }};
    }

    public DumpOuttakeAction dumpOuttakeAction() {
        return new DumpOuttakeAction(this);
    }

    public ExtendOuttakeAction extendOuttakeAction(VerticalSlideLevel slideLevel) {
        return new ExtendOuttakeAction(this, slideLevel);
    }

    public ExtendOuttakeAction extendOuttakeToDetectedPosition(TeamMarkerDetector detector) {
        return new ExtendOuttakeAction(this, detector.getLocation().slideLevel()) {
            @Override
            public void tick() {
                this.slideLevel = detector.getLocation().slideLevel();
                super.tick();
            }
        };
    }

}
