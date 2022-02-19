package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Intake.INTAKE_RETRACT_TIME;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Intake.RING_BUFFER_CAPACITY;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AnalogDistance;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.collections4.BoundedCollection;
import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;

public class IntakeModule implements Module, Telemeter {
    public static final double INTAKE_RIGHT_EXTENDED_POS = 0.0168;
    public static final double INTAKE_RIGHT_RETRACTED_POS = 0.6860951;
    public static final double INTAKE_LEFT_EXTENDED_POS = 0.949761;
    public static final double INTAKE_LEFT_RETRACTED_POS = 0.289751;

    private static final double HOLD_POWER = 1;

    private final CircularFifoQueue<Double> intakeRpmRingBuffer = new CircularFifoQueue<>(RING_BUFFER_CAPACITY);

    private final DcMotorEx intakeMotor;
    private final Servo extenderLeft;
    private final Servo extenderRight;

    private final AnalogDistance distanceSensor;

    private Robot robot;

    // states
    public volatile IntakePosition intakePosition = IntakePosition.EXTENDED;
    public volatile double intakePower;

    private Long intakerRetractionStartTime;
    private volatile boolean intakeOccupied = false;
    private volatile boolean newIntakeOccupied = false;
    private boolean started = false;

    List<Double> avgRPMs;

    Queue<Boolean> queue = new CircularFifoQueue<>(10);

    private boolean mineralInIntake() {
        // needs to be tuned
        if (getDistanceSensorReading() < 42) {
            Log.v("intake", "true");
            queue.add(true);
            return true;
        } else if (getDistanceSensorReading() < 70) {
            queue.add(true);

            Boolean[] queueArray = queue.toArray(new Boolean[]{});
            for (int i = 0; i < queueArray.length; i++) {
                if (!queueArray[i]) {
                    return false;
                }
            }
            return true;
        }
        queue.add(false);
        return false;
    }

    private boolean hasDecelerated() {
        if (avgRPMs.size() < 2)
            return false;
        return avgRPMs.get(avgRPMs.size() - 1) < avgRPMs.get(0);
    }

    public static <T> void fill(BoundedCollection<T> collection, T value) {
        for (int i = 0; i < collection.maxSize(); i++) {
            collection.add(value);
        }
    }

    private boolean isOn;

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

        robot.telemetryDump.registerTelemeter(this);
    }

    public void setIntakePosition(IntakePosition intakePosition) {
        synchronized (extenderLeft) {
            synchronized (extenderRight) {
                this.intakePosition = intakePosition;
                switch (intakePosition) {
                    case EXTENDED:
                        extenderLeft.setPosition(INTAKE_LEFT_EXTENDED_POS);
                        extenderRight.setPosition(INTAKE_RIGHT_EXTENDED_POS);
                        break;
                    case RETRACTED:
                        extenderLeft.setPosition(INTAKE_LEFT_RETRACTED_POS);
                        extenderRight.setPosition(INTAKE_RIGHT_RETRACTED_POS);
                        break;
                }
            }
        }
    }

    public void update() {
        synchronized (intakeRpmRingBuffer) {
            if (!robot.isOpModeActive()) {
                setIntakePosition(IntakePosition.RETRACTED);
            } else if (!started) {
                setIntakePosition(IntakePosition.EXTENDED);
                started = true;
            }

//            doOccupationStatusProcessing();

            // If we're done retracting (or the driver has pushed
            // the intake joystick up, we should re-extend
            if (inRetractionState() &&
                    (intakePower < 0 || doneRetracting()))
                startIntakeExtension();

            // If the intake is occupied and we haven't
            // started retracting yet, we should do that.
            newIntakeOccupied = mineralInIntake();
            if (newIntakeOccupied && !inRetractionState())
                if (robot.outtakeModule.collapsed())
                    startIntakeRetraction();

            intakeMotor.setPower(
                    inRetractionState() ? HOLD_POWER : intakePower
            );
        }
    }

    public void requestRetraction() {
        if (!inRetractionState())
            startIntakeRetraction();
    }

    private void startIntakeRetraction() {
        setIntakePosition(IntakePosition.RETRACTED);
        intakerRetractionStartTime = SystemClock.elapsedRealtime();
    }

    private boolean doneRetracting() {
        return SystemClock.elapsedRealtime() - intakerRetractionStartTime
                >= INTAKE_RETRACT_TIME;
    }

    private synchronized void startIntakeExtension() {
        if (robot != null) {
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
        }
        intakeOccupied = false;
        intakerRetractionStartTime = null;
        setIntakePosition(IntakePosition.EXTENDED);
    }

    public boolean inRetractionState() {
        return intakerRetractionStartTime != null;
    }

    public double getDistanceSensorReading() {
        return distanceSensor.getSensorReading();
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

//        data.add("retract: " + inRetractionState());
        data.add(String.format(Locale.US, "Intake position:  %s", intakePosition));
        data.add(String.format(Locale.US, "Intake occupied:  %b", intakeOccupied));
//        data.add(String.format(Locale.US, "sd:  %f", lastSd));
//        data.add(String.format(Locale.US, "buf len:  %d", intakeRpmRingBuffer.size()));

        data.add("--");
        data.add(String.format(Locale.US, "Distance sensor reading: %s", getDistanceSensorReading()));
        data.add(String.format(Locale.US, "Mineral is in intake: %b", newIntakeOccupied));

        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
