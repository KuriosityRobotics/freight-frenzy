package com.kuriosityrobotics.firstforward.robot.modules.leds;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import java.util.ArrayList;
import java.util.List;

public class LEDModule implements Module, Telemeter {
    BlinkinPattern VUF_INITING = BlinkinPattern.DARK_BLUE;
    BlinkinPattern INTAKE_OCCUPIED = BlinkinPattern.GREEN;
    BlinkinPattern IDLE = BlinkinPattern.BREATH_RED;

    BlinkinPattern VUF_USED = BlinkinPattern.BLUE;
    BlinkinPattern VUF_SAW = BlinkinPattern.GOLD;

    private static final long SHOW_VUF = 500;

    // modules
    Robot robot;
    IntakeModule intake;

    //servos
    private final RevBlinkinLedDriver led;

    public LEDModule(Robot robot) {
        led = robot.getHardwareMap().get(RevBlinkinLedDriver.class, "LED");

        this.robot = robot;
        this.intake = robot.getIntakeModule();
    }

    public void update() {
        if (intake != null && intake.hasMineral()) {
            led.setPattern(INTAKE_OCCUPIED);
        } else if (!robot.getVisionThread().isStarted()) {
            led.setPattern(VUF_INITING);
        } else if (SystemClock.elapsedRealtime() <= robot.getVisionThread().getVuforiaLocalizationConsumer().getLastAcceptedTime() + SHOW_VUF) {
            led.setPattern(VUF_USED);
        } else if (SystemClock.elapsedRealtime() <= robot.getVisionThread().getVuforiaLocalizationConsumer().getLastDetectedTime() + SHOW_VUF) {
            led.setPattern(VUF_SAW);
        } else {
            led.setPattern(IDLE);
        }
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
    public int getShowIndex() {
        return 1;
    }

    @Override
    public int maxFrequency() {
        return 2;
    }
}
