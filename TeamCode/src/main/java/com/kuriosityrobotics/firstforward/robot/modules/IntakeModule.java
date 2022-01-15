package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Intake.*;

import android.annotation.SuppressLint;
import android.os.SystemClock;
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
import java.util.List;
import java.util.Locale;

public class IntakeModule implements Module, Telemeter {
    public volatile double lastSd = 0;

    private final CircularFifoQueue<Double> intakeRpmRingBuffer = new CircularFifoQueue<>(RING_BUFFER_CAPACITY);

    private final DcMotorEx intakeMotor;
    private final Servo extenderLeft;
    private final Servo extenderRight;

    private Robot robot;

    // states
    public volatile IntakePosition intakePosition = IntakePosition.EXTENDED;
    public volatile boolean retractIntake = false;
    public volatile double intakePower;

    private Long intakerRetractionStartTime;
    private volatile boolean intakeOccupied = false;
    private boolean started = false;

    private boolean hasOccupationStatusChanged() {
        if (intakePower < 0.1 || intakeRpmRingBuffer.isEmpty())
            return false;
        List<Double> avgRPMs = new ArrayList<>();
        double last = 0;
        for (int i = 0; i < RING_BUFFER_CAPACITY; i++) {
            if (i % 10 == 0 || i == RING_BUFFER_CAPACITY - 1) {
                avgRPMs.add(last / 10);
                last = 0;
            } else {
                last += intakeRpmRingBuffer.get(i);
            }
        }

        double sd = MathUtil.sd(avgRPMs);
//        var sd = MathUtil.sd(intakeRpmRingBuffer);
        lastSd = sd;
        return INTAKE_OCCUPIED_SD < sd && sd < INTAKE_DEACCEL_SD;
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
        EXTENDED,
        RETRACTED
    }

    public IntakeModule(Robot robot, boolean isOn) {
        this(robot.hardwareMap, isOn);
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);
    }

    public IntakeModule(HardwareMap map, boolean isOn) {
        this.isOn = isOn;

        this.extenderLeft = map.tryGet(Servo.class, "intakeExtenderLeft");
        this.extenderRight = map.tryGet(Servo.class, "intakeExtenderRight");
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

    public void update() {
        synchronized (intakeRpmRingBuffer) {
            if (!robot.isOpModeActive()) {
                setIntakePosition(IntakePosition.RETRACTED);
            } else if (!started) {
                setIntakePosition(IntakePosition.EXTENDED);
                started = true;
            }

            if (intakePower < 0 && intakerRetractionStartTime != null) {
                startIntakeExtension();
            }

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
                startIntakeExtension();
            } else if (intakeOccupied && !retractIntake) {
                retractIntake = true;
            }

            if (retractIntake && intakerRetractionStartTime == null) {
                if (robot.outtakeModule.readyForIntake()) {
                    setIntakePosition(IntakePosition.RETRACTED);
                    intakerRetractionStartTime = SystemClock.elapsedRealtime();
                } else {
                    OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.DOWN;
                }
            }

            if (intakerRetractionStartTime != null) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(intakePower);
            }
        }
    }

    private synchronized void startIntakeExtension() {
        if (robot != null) {
            robot.outtakeModule.hopperOccupied();
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.TOP;
        }
        intakeOccupied = false;
        intakerRetractionStartTime = null;
        setIntakePosition(IntakePosition.EXTENDED);
        retractIntake = false;
    }

    public boolean isIntakeRetracting() {
        return intakerRetractionStartTime != null;
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add(String.format(Locale.US, "Intake speed (RPM): %f", getRPM()));
        data.add("retract: " + retractIntake);
        data.add(String.format(Locale.US, "Intake position:  %s", intakePosition));
        data.add(String.format(Locale.US, "Intake occupied:  %b", intakeOccupied));
        data.add(String.format(Locale.US, "sd:  %f", lastSd));
        data.add(String.format(Locale.US, "buf len:  %d", intakeRpmRingBuffer.size()));

        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
