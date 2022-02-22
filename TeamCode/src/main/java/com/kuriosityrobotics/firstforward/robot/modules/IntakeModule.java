package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AnalogDistance;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.ArrayList;
import java.util.Locale;
import java.util.Queue;

public class IntakeModule implements Module, Telemeter {
    public static final double INTAKE_RIGHT_EXTENDED_POS = 0.0168;
    public static final double INTAKE_RIGHT_IDLE_POS = 0.5121062;
    public static final double INTAKE_RIGHT_RETRACTED_POS = 0.6860951;
    public static final double INTAKE_LEFT_EXTENDED_POS = 0.949761;
    public static final double INTAKE_LEFT_RETRACTED_POS = 0.289751;
    public static final double INTAKE_LEFT_IDLE_POS = INTAKE_LEFT_RETRACTED_POS + (INTAKE_RIGHT_RETRACTED_POS - INTAKE_RIGHT_IDLE_POS);

    private static final double CLOSE_DISTANCE_THRESHOLD = 42;
    private static final double FAR_DISTANCE_THRESHOLD = 70;

    public static final long INTAKE_EXTEND_TIME = 1500;
    public static final long INTAKE_RETRACT_TIME = 1000;

    private static final double HOLD_POWER = 1;

    // states
    public volatile double intakePower;
    public volatile IntakePosition targetIntakePosition;

    // hardware
    private final DcMotorEx intakeMotor;
    private final Servo extenderLeft;
    private final Servo extenderRight;

    private final AnalogDistance distanceSensor;

    private final Robot robot;

    // helpers
    private long lastTransitionTime;
    private IntakePosition lastPosition;
    private boolean hasMineral = false;
    private boolean started = false;
    private boolean doneTransitioning = false;
    private IntakePosition currentPosition;

    Queue<Double> distanceReadings = new CircularFifoQueue<>(15);

    private final boolean isOn;

    public enum IntakePosition {
        EXTENDED,
        RETRACTED
    }

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;

        this.isOn = isOn;

        this.extenderLeft = robot.hardwareMap.servo.get("extenderLeft");
        this.extenderRight = robot.hardwareMap.servo.get("extenderRight");
        this.intakeMotor = (DcMotorEx) robot.hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.distanceSensor = new AnalogDistance(robot.hardwareMap.get(AnalogInput.class, "distance"));

        this.targetIntakePosition = IntakePosition.RETRACTED;
        this.lastPosition = targetIntakePosition;
        this.lastTransitionTime = 0;

        robot.telemetryDump.registerTelemeter(this);
    }

    public void update() {
        if (!robot.isOpModeActive() && !started) {
            targetIntakePosition = IntakePosition.RETRACTED;
        } else if (!started) {
            targetIntakePosition = IntakePosition.EXTENDED;
            started = true;
        }

        if (started) {
            checkForStateChange();

            if (atTargetPosition() && !doneTransitioning) {
                if (targetIntakePosition == IntakePosition.RETRACTED) {
                    robot.outtakeModule.defaultFullExtend();
                }
                doneTransitioning = true;
            }

            if (intakePower > 0 && atPosition(IntakePosition.RETRACTED)) {
                targetIntakePosition = IntakePosition.EXTENDED;
            }

            // If the intake is occupied and we haven't
            // started retracting yet, we should do that.
            hasMineral = mineralInIntake();
            if (hasMineral && atPosition(IntakePosition.EXTENDED)) {
                if (robot.outtakeModule.collapsed()) {
                    targetIntakePosition = IntakePosition.RETRACTED;
                }
            }

            checkForStateChange();

            if (targetIntakePosition == IntakePosition.RETRACTED && !atTargetPosition()) {
                intakeMotor.setPower(HOLD_POWER);
            } else {
                intakeMotor.setPower(intakePower);
            }
        }

        switch (targetIntakePosition) {
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
        }
    }

    private void checkForStateChange() {
        if (lastPosition != targetIntakePosition) {
            lastTransitionTime = SystemClock.elapsedRealtime();
            doneTransitioning = false;
        }
        this.lastPosition = targetIntakePosition;
    }

    public boolean atPosition(IntakePosition position) {
        boolean rightPosition = position == targetIntakePosition;

        return rightPosition && !transitioning();
    }

    public boolean atTargetPosition() {
        return atPosition(targetIntakePosition);
    }

    public boolean transitioning() {
        long transitionTime = targetIntakePosition == IntakePosition.EXTENDED ? INTAKE_EXTEND_TIME : INTAKE_RETRACT_TIME;
        boolean transitioning = lastTransitionTime + transitionTime > SystemClock.elapsedRealtime();
        return transitioning;
    }

    public boolean retracting() {
        Log.v("in", "pos: " + targetIntakePosition);
        Log.v("in", "trans: " + transitioning());
        return targetIntakePosition == IntakePosition.RETRACTED && transitioning();
    }

    public boolean hasMineral() {
        return hasMineral;
    }

    private boolean mineralInIntake() {
        double reading = distanceSensor.getSensorReading();
        distanceReadings.add(reading);

        // needs to be tuned
        if (reading < CLOSE_DISTANCE_THRESHOLD) {
            // if last 4 are all positives it's a go
            Object[] queueArray = distanceReadings.toArray();
            for (int i = queueArray.length - 1; i > Math.max(queueArray.length - 5, 0); i--) {
                if (((double) queueArray[i]) > CLOSE_DISTANCE_THRESHOLD) {
                    return false;
                }
            }
            return true;
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
            return true;
        }
        return false;
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add(String.format(Locale.US, "Intake position:  %s", targetIntakePosition));
        data.add("At pos? " + atTargetPosition());

        data.add("--");
        data.add(String.format(Locale.US, "Mineral is in intake: %b", hasMineral));

        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
