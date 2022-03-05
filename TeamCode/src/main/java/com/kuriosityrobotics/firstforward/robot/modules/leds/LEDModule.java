package com.kuriosityrobotics.firstforward.robot.modules.leds;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class LEDModule implements Module, Telemeter {
    private long updateDuration = 0;
    private long timeOfLastUpdate = 0;

    RevBlinkinLedDriver.BlinkinPattern VUF_INITING = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
    RevBlinkinLedDriver.BlinkinPattern INTAKE_OCCUPIED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    RevBlinkinLedDriver.BlinkinPattern IDLE = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

    // modules
    Robot robot;
    IntakeModule intake;

    //servos
    private final RevBlinkinLedDriver led;

    public LEDModule(Robot robot) {
        led = robot.hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        this.robot = robot;
        this.intake = robot.intakeModule;
    }

    public void update() {
        if (intake != null && intake.hasMineral()) {
            led.setPattern(INTAKE_OCCUPIED);
        } else if (!robot.visionThread.started) {
            led.setPattern(VUF_INITING);
        } else {
            led.setPattern(IDLE);
        }

        long currentTime = SystemClock.elapsedRealtime();
        updateDuration = currentTime - timeOfLastUpdate;
        timeOfLastUpdate = currentTime;
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "LED Module";
    }

    @Override
    public Iterable<String> getTelemetryData() {
        return new ArrayList<>() {{add("Update Time" + updateDuration);
            add("--");}};
    }

    @Override
    public int getShowIndex() {
        return 1;
    }
}
