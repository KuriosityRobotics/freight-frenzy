package com.kuriosityrobotics.firstforward.robot.modules.intake;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AnalogDistance;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.ArrayList;
import java.util.Locale;

public class IntakeModule implements Module, Telemeter {
    public static final double INTAKE_RIGHT_EXTENDED_POS = 0.0168;
    public static final double INTAKE_RIGHT_IDLE_POS = 0.5121062;
    public static final double INTAKE_RIGHT_RETRACTED_POS = 0.6860951;
    public static final double INTAKE_LEFT_EXTENDED_POS = 0.949761;
    public static final double INTAKE_LEFT_RETRACTED_POS = 0.289751;
    public static final double INTAKE_LEFT_IDLE_POS = INTAKE_LEFT_RETRACTED_POS + (INTAKE_RIGHT_RETRACTED_POS - INTAKE_RIGHT_IDLE_POS);

    private static final double CLOSE_DISTANCE_THRESHOLD = 38;
    private static final double FAR_DISTANCE_THRESHOLD = 70;

    public static final long INTAKE_EXTEND_TIME = 750;
    public static final long INTAKE_RETRACT_TIME = 1150;

    private static final double HOLD_POWER = 0.8;

    // states
    public volatile double intakePower;
    public volatile IntakePosition targetIntakePosition;

    public boolean enableAutoExtend = true;

    private final DcMotorEx intakeMotor;
    private final Servo extenderLeft;
    private final Servo extenderRight;
    private final OuttakeModule outtakeModule;

    private final AnalogDistance distanceSensor;

    // helpers
    private IntakePosition transitionTo;
    private long transitionTime;
    private boolean wasDoneTransitioning;
    public boolean retracted = false;

    private boolean hasMineral;

    public volatile boolean newMineral = false;
    CircularFifoQueue<Double> distanceReadings = new CircularFifoQueue<>(15);

    public enum IntakePosition {
        EXTENDED,
        RETRACTED,
        STAY_RETRACTED
    }

    public IntakeModule(HardwareMap hardwareMap, OuttakeModule outtakeModule) {
        this.outtakeModule = outtakeModule;

        this.extenderLeft = hardwareMap.servo.get("extenderLeft");
        this.extenderRight = hardwareMap.servo.get("extenderRight");
        this.intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.distanceSensor = new AnalogDistance(hardwareMap.get(AnalogInput.class, "distance"));

        this.targetIntakePosition = IntakePosition.RETRACTED;
        this.transitionTo = targetIntakePosition;
        this.transitionTime = 0;
        this.wasDoneTransitioning = true;

        this.hasMineral = false;
    }

    public void update() {
        // listen for when we just finished retracting to command the outtake to extend.
        if (atTargetPosition() && !wasDoneTransitioning) {
            if (transitionTo == IntakePosition.RETRACTED && enableAutoExtend) {
                outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
            }
            wasDoneTransitioning = true;
        }

        // listen for state change
        if (targetIntakePosition != transitionTo) {
            transitionIntake(targetIntakePosition);
        }

        hasMineral = mineralInIntake();
        // if we're done transitioning, there are a handful of listeners that apply
        if (!transitioning()) {
            // if we're done retracting but trying to intake
            if (intakePower > 0 && transitionTo == IntakePosition.RETRACTED) {
                targetIntakePosition = IntakePosition.EXTENDED;
                transitionIntake(targetIntakePosition);
            }

            // if we're done extending and there's a mineral in the intake
            if (hasMineral && transitionTo == IntakePosition.EXTENDED) {
                newMineral = true;
                if (outtakeModule.collapsed()) {
                    intakePower = 0;
                    targetIntakePosition = IntakePosition.RETRACTED;
                    transitionIntake(targetIntakePosition);
                }
            }
        }

        boolean atTarget = atTargetPosition();
        // set motor power
        if (transitionTo == IntakePosition.RETRACTED && !atTarget) {
            intakeMotor.setPower(HOLD_POWER);
        } else {
            intakeMotor.setPower(intakePower);
        }

        // set intake position
        switch (transitionTo) {
            case EXTENDED:
                extenderLeft.setPosition(INTAKE_LEFT_EXTENDED_POS);
                extenderRight.setPosition(INTAKE_RIGHT_EXTENDED_POS);
                break;
            case RETRACTED:
                if (atTargetPosition()) {
                    extenderLeft.setPosition(INTAKE_LEFT_IDLE_POS);
                    extenderRight.setPosition(INTAKE_RIGHT_IDLE_POS);
                } else {
                    extenderLeft.setPosition(INTAKE_LEFT_RETRACTED_POS);
                    extenderRight.setPosition(INTAKE_RIGHT_RETRACTED_POS);
                }
                break;
            case STAY_RETRACTED:
                extenderLeft.setPosition(INTAKE_LEFT_RETRACTED_POS);
                extenderRight.setPosition(INTAKE_RIGHT_RETRACTED_POS);
                break;
        }

    }

    private void transitionIntake(IntakePosition position) {
        this.transitionTo = position;

        if (transitionTo == IntakePosition.RETRACTED) {
            retracted = true;
        }

        this.transitionTime = SystemClock.elapsedRealtime();
        this.wasDoneTransitioning = false;
    }

    public boolean transitioning() {
        long transitionDuration = transitionTo == IntakePosition.EXTENDED ? INTAKE_EXTEND_TIME : INTAKE_RETRACT_TIME;
        return this.transitionTime + transitionDuration > SystemClock.elapsedRealtime();
    }

    public boolean atPosition(IntakePosition position) {
        boolean rightPosition = position == transitionTo;

        return rightPosition && !transitioning();
    }

    public boolean atTargetPosition() {
        return atPosition(targetIntakePosition);
    }

    public boolean hasMineral() {
        return hasMineral;
    }

    public boolean spinning;

    private boolean mineralInIntake() {
        spinning = intakeSpinning();

        double reading = distanceSensor.getSensorReading();
        distanceReadings.add(reading);

        if (reading < CLOSE_DISTANCE_THRESHOLD) {
            // if last 4 are all positives it's a go
            Object[] queueArray = distanceReadings.toArray();
            for (int i = queueArray.length - 1; i > Math.max(queueArray.length - 5, 0); i--) {
                if (((double) queueArray[i]) > CLOSE_DISTANCE_THRESHOLD) {
                    return false;
                }
            }
            return spinning;
        } else if (reading < FAR_DISTANCE_THRESHOLD) {
            // if last 10 are all positives it's a go
            Object[] queueArray = distanceReadings.toArray();
            int start = Math.max(queueArray.length - 1, 0);
            int limit = Math.max(queueArray.length - 10, 0);
            for (int i = start; i > limit; i--) {
                if (((double) queueArray[i]) > FAR_DISTANCE_THRESHOLD) {
                    return false;
                }
            }
            return spinning;
        }
        return false;
    }

    private boolean intakeSpinning() {
        return Math.abs(intakePower) > .5;
    }

    public boolean isOn() {
        return true;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

//        data.add(String.format(Locale.US, "Intake position:  %s", targetIntakePosition));
        data.add("Intake position: " + targetIntakePosition);
        data.add("At pos? " + atTargetPosition());

        data.add("--");
//        data.add(String.format(Locale.US, "Mineral is in intake: %b", hasMineral));
        data.add("Mineral is in intake: " + hasMineral);

        return data;
    }

    public String getName() {
        return "IntakeModule";
    }

    public IntakeAction intakeAction() {
        return new IntakeAction(this);
    }

    public IntakePowerAction intakePowerAction(double power) {
        return new IntakePowerAction(this, power);
    }
}