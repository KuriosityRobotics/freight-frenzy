package com.kuriosityrobotics.firstforward.robot.modules;

import android.annotation.SuppressLint;
import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.MathUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.collections4.BoundedCollection;
import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.ArrayList;

public class IntakeModule implements Module, Telemeter {
    private static final double RPM_EPSILON = 60;

    // PPR = Ticks per Revolution(abrv different but doesn't matter)
    private static final double GOBILDA_1620_PPR = 103.8;
    private static final double INTAKE_OCCUPIED_SD = 100;

    // TODO: Tune
    public static final double INTAKE_EXTEND_TIME = 1000;

    public static final double INTAKE_RIGHT_EXTENDED_POS = 0.512;
    public static final double INTAKE_RIGHT_RETRACTED_POS = 0.87614;
    public static final double INTAKE_LEFT_EXTENDED_POS = 0.8691;
    public static final double INTAKE_LEFT_RETRACTED_POS = 0.5;

    private static final int RING_BUFFER_CAPACITY = 100;
    private final CircularFifoQueue<Double> intakeRpmRingBuffer = new CircularFifoQueue<>(RING_BUFFER_CAPACITY);

    private volatile double intakePower;
    private final DcMotorEx intakeMotor;
    private final Servo extenderLeft;
    private final Servo extenderRight;

    public volatile IntakePosition intakePosition = IntakePosition.EXTENDED;

    private Long intakerRetractionStartTime;
    private volatile boolean intakeOccupied = false;

    private boolean hasOccupationStatusChanged() {
        if (intakePower < 0.1 || intakeRpmRingBuffer.isEmpty())
            return false;
        var sd = MathUtil.sd(intakeRpmRingBuffer);
        Log.v("intake", "" + sd);
        return INTAKE_OCCUPIED_SD < sd && sd < 300;
    }

    private boolean hasDecelerated() {
        if (intakeRpmRingBuffer.isEmpty())
            return false;
        return intakeRpmRingBuffer.get(intakeRpmRingBuffer.size() - 1) < intakeRpmRingBuffer.get(0);
    }

    public static <T> void fill(BoundedCollection<T> collection, T value) {
        for (int i = 0; i < collection.maxSize(); i++) {
            collection.add(value);
        }
    }

    private boolean isOn;

    public enum IntakePosition {
        EXTENDED, RETRACTED
    }

    public IntakeModule(Robot robot, boolean isOn) {
        this(robot.hardwareMap, isOn);
        robot.telemetryDump.registerTelemeter(this);
    }

    public IntakeModule(HardwareMap map, boolean isOn) {
        this.isOn = isOn;

        this.extenderLeft = map.tryGet(Servo.class, "extenderLeft");
        this.extenderRight = map.tryGet(Servo.class, "extenderRight");
        this.intakeMotor = (DcMotorEx) map.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getRPM() {
        synchronized (intakeRpmRingBuffer) {
            return intakeRpmRingBuffer.get(intakeRpmRingBuffer.size() - 1); // hopefully this is the most, not least, recent element
        }
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

    public void setPower(double power) {
        if (power < 0 && intakerRetractionStartTime != null)
            startIntakeExtension();

        if (intakerRetractionStartTime == null && power != this.intakePower) {
            this.intakePower = power;
            intakeMotor.setPower(power);
        }
    }

    public void update() {
        synchronized (intakeRpmRingBuffer) {
//            Log.v("Intake Module", "Intake motor velo: " + intakeMotor.getVelocity());
            intakeRpmRingBuffer.add(intakeMotor.getVelocity() * 60 / GOBILDA_1620_PPR);

            // since it's a ring buffer we could make the following more efficient
            // but it's literally ten elements so it doesn't matter
            if (getRPM() < RPM_EPSILON) {
                intakeOccupied = false;
            } else if (hasOccupationStatusChanged()) {
                intakeOccupied = hasDecelerated();
                fill(intakeRpmRingBuffer, getRPM()); // heinous hackery to set the stddev to 0
            }

            if (intakerRetractionStartTime != null && SystemClock.elapsedRealtime() - intakerRetractionStartTime >= INTAKE_EXTEND_TIME) {
                setPower(intakePower / .75);
                startIntakeExtension();
            } else if (intakeOccupied) {
                startIntakeRetraction();
            }
        }
    }

    private synchronized void startIntakeExtension() {
        intakeOccupied = false;
        intakerRetractionStartTime = null;
        setIntakePosition(IntakePosition.EXTENDED);
    }

    public void startIntakeRetraction() {
        if (intakerRetractionStartTime == null) {
            setIntakePosition(IntakePosition.RETRACTED);
            setPower(.75);
            intakerRetractionStartTime = SystemClock.elapsedRealtime();
        }
    }

    public boolean isOn() {
        return isOn;
    }

    @SuppressLint("DefaultLocale") // please java shut the Gell Up
    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add(String.format("Intake speed (RPM): %f", getRPM()));
        data.add(String.format("Intake position:  %s", intakePosition));
        data.add(String.format("Intake occupied:  %b", intakeOccupied));
        data.add(String.format("Ready for outake:  %b", readyForOutake()));

        return data;
    }

    public boolean isRetracted() {
        return this.intakePosition == IntakePosition.RETRACTED;
    }

    public boolean readyForOutake() {
        return isRetracted() && intakeOccupied;
    }

    public String getName() {
        return "IntakeModule";
    }
}
