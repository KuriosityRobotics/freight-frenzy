package com.kuriosityrobotics.firstforward.robot.modules.outtake;

import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.DUMP;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.EXTEND;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.PARTIAL_EXTEND;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.HUBS;
import static java.lang.Math.abs;

import android.os.SystemClock;
import android.util.Log;
import static java.lang.Math.round;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class OuttakeModule implements Module, Telemeter {
    LocationProvider locationProvider;

    // states
    public VerticalSlideLevel targetSlideLevel = VerticalSlideLevel.TOP;
    public TurretPosition targetTurret = TurretPosition.STRAIGHT;
    public LinkagePosition targetLinkage = LinkagePosition.EXTEND;
    public PivotPosition targetPivot = PivotPosition.OUT;
    public OuttakeState targetState;

    private OuttakeState currentState;

    //time constants
    private static final long EXTEND_TIME = 400;
    private static final long DUMP_TIME = 300;
    private static final long TURRET_TIME = 150; // if the turret isn't already straight

    private static final double CLAMP_INTAKE = 0.85465,
            CLAMP_CLAMP = 0.767,
            CLAMP_RELEASE = 0.90492;

    private final double EXTENDED_TURRET_OFFSET_Y = 14.3;

    // from the perspective of looking out from the back of the robot
    public enum TurretPosition {
        STRAIGHT(0.492083),
        // raised up to .502746
        //raised down to .482746
        RIGHT(.78988),
        LEFT(.186781),
        SHARED_RIGHT(0.55),
        SHARED_LEFT(0.426),
        SHARED_RIGHT_MORE_EXTREME_ANGLE(0.65),
        SHARED_LEFT_MORE_EXTREME_ANGLE(0.326),
        ALLIANCE_LOCK(0.5);

        private final double position;

        TurretPosition(double position) {
            this.position = position;
        }
    }

    public enum VerticalSlideLevel {
        CAP(-1400),
        CAP_DROP(-1200),
        TOP_TOP(-1200),
        TOP(-1000),
        MID(-427),
        SHARED(-200),
        DOWN(-2),
        DOWN_NO_EXTEND(-2);

        private final int position;

        VerticalSlideLevel(int position) {
            this.position = position;
        }
    }

    public enum PivotPosition {
        IN(.0060539),
        OUT(.993),
        UP(0.5),
        CAP_PICKUP(0.9874),
        CAP_DROP(0.816);

        private final double position;

        PivotPosition(double pos) {
            this.position = pos;
        }
    }

    public enum LinkagePosition {
        RETRACT(.140102),
        EXTEND(.8777921),
        PARTIAL_EXTEND(0.45);

        private final double position;

        LinkagePosition(double pos) {
            this.position = pos;
        }
    }

    public enum OuttakeState {
        PARTIAL_EXTEND(400),
        RAISE(0),
        EXTEND(EXTEND_TIME),
        DUMP(DUMP_TIME),
        RETRACT(EXTEND_TIME),
        TURRET_IN(0),
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
                    return true;
                } else {
                    return timerComplete;
                }
            case RETRACT:
                if (targetSlideLevel == VerticalSlideLevel.TOP || targetSlideLevel == VerticalSlideLevel.TOP_TOP || targetSlideLevel == VerticalSlideLevel.CAP) {
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

    private boolean timerComplete() {
        long currentTime = System.currentTimeMillis();
        boolean timerComplete = currentTime >= transitionTime + currentState.completionTime;

        return timerComplete;
    }

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

        linkage = hardwareMap.servo.get("outtakeLinkage");
        pivot = hardwareMap.servo.get("outtakePivot");
        clamp = hardwareMap.servo.get("outtakeClamp");
        turret = hardwareMap.servo.get("outtakeTurret");

        //use this when outtake is fucked
        /*linkage = hardwareMap.servo.get("nothingServo");
        pivot = hardwareMap.servo.get("nothingServo");
        clamp = hardwareMap.servo.get("nothingServo");
        turret = hardwareMap.servo.get("nothingServo");*/

        slide = (DcMotorEx) hardwareMap.dcMotor.get("lift");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12, 0, 0, 20));

        clamp.setPosition(CLAMP_INTAKE);
        pivot.setPosition(PivotPosition.IN.position);
        linkage.setPosition(LinkagePosition.RETRACT.position);

        this.targetState = COLLAPSE;
        this.currentState = COLLAPSE;
    }

    public void resetSlides() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void skipToCollapse() {
        if (targetState == COLLAPSE) {
            return;
        }

        transitionTime = 0;
        this.targetState = COLLAPSE;
        this.currentState = DUMP;
    }

    String lastRan = "";

    public void update() {
        boolean skipState = currentState == COLLAPSE;
        if ((phaseComplete() || skipState) && currentState != targetState) {
            lastRan = currentState.name();

            currentState = OuttakeState.values()[currentState.ordinal() + 1 >= OuttakeState.values().length ? 0 : currentState.ordinal() + 1];

            switch (this.currentState) {
                case PARTIAL_EXTEND:
                    clamp.setPosition(CLAMP_CLAMP);
                    linkage.setPosition(LinkagePosition.PARTIAL_EXTEND.position);
                    break;
                case RAISE:
                    linkage.setPosition(LinkagePosition.PARTIAL_EXTEND.position);
                    clamp.setPosition(CLAMP_CLAMP);
                    slide.setTargetPosition(targetSlideLevel.position);
                    break;
                case EXTEND:
                    pivot.setPosition(targetPivot.position);
                    break;
                case DUMP:
                    clamp.setPosition(CLAMP_RELEASE);
                    break;
                case RETRACT:
                    linkage.setPosition(LinkagePosition.RETRACT.position);
                    break;
                case TURRET_IN:
                    pivot.setPosition(PivotPosition.UP.position);
                    turret.setPosition(TurretPosition.STRAIGHT.position);
                    break;
                case COLLAPSE:
                    clamp.setPosition(CLAMP_CLAMP);
                    pivot.setPosition(PivotPosition.IN.position);
                    linkage.setPosition(LinkagePosition.RETRACT.position);
                    slide.setTargetPosition(VerticalSlideLevel.DOWN.position);
                    break;
            }

            transitionTime = System.currentTimeMillis();
        }

//        if (slide.getCurrentPosition() > 10) {
//            resetSlides();
//        }

        // if current position is higher than the target
        if (currentState == COLLAPSE || currentState == PARTIAL_EXTEND) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0);

            if (timerComplete()) {
                clamp.setPosition(CLAMP_INTAKE);
            }
        } else {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition(targetSlideLevel.position);
            slide.setPower(1);
        }

        if (currentState == EXTEND) {
            linkage.setPosition(targetLinkage.position);

            if (atTargetState()) {
                pivot.setPosition(targetPivot.position);

                double targetTurretServoPosition = targetTurret.position;
                if (targetTurret == TurretPosition.ALLIANCE_LOCK) {
                    Pose robotPose = locationProvider.getPose();
                    Pose turretPose = new Pose(
                            robotPose.x - EXTENDED_TURRET_OFFSET_Y * Math.sin(robotPose.heading),
                            robotPose.y - EXTENDED_TURRET_OFFSET_Y * Math.cos(robotPose.heading),
                            robotPose.heading + Math.PI
                    );

                    Point targetHub = turretPose.nearestPoint(HUBS);

                    double targetTurretHeading = turretPose.relativeHeadingToPoint(targetHub);
                    targetTurretHeading = Range.clip(targetTurretHeading, -Math.PI / 2, Math.PI / 2);

                    targetTurretServoPosition = turretHeadingToServoPos(targetTurretHeading);
                }
                turret.setPosition(targetTurretServoPosition);

                if (targetSlideLevel == VerticalSlideLevel.CAP_DROP) {
                    clamp.setPosition(CLAMP_RELEASE);
                } else {
                    clamp.setPosition(CLAMP_CLAMP);
                }
            }
        }
    }

    private double turretHeadingToServoPos(double turretHeading) {
        return TurretPosition.STRAIGHT.position + turretHeading * (TurretPosition.RIGHT.position - TurretPosition.LEFT.position) / Math.PI;
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
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("Target State: " + targetState);
            add("State:  " + currentState);
//            add("last:  " + lastRan);
            add("slideLevel: " + targetSlideLevel.name());
            add("Turret: " + targetTurret.name());
            add("Linkage: " + targetLinkage.name());
//            add("pivot postiong: " + pivot.getPosition());
//            add("--");
//            add("current slide:  " + slide.getCurrentPosition());
        }};
    }

    @Override
    public int getShowIndex() {
        return 1;
    }

    public DumpOuttakeAction dumpOuttakeAction() {
        return new DumpOuttakeAction(this);
    }

    public ExtendOuttakeAction extendOuttakeAction(VerticalSlideLevel slideLevel) {
        return new ExtendOuttakeAction(this, slideLevel, TurretPosition.ALLIANCE_LOCK);
    }

    public ExtendOuttakeAction extendOuttakeAction(VerticalSlideLevel slideLevel, TurretPosition turretPosition) {
        return new ExtendOuttakeAction(this, slideLevel, turretPosition);
    }

}
