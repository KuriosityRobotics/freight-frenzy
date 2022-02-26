package com.kuriosityrobotics.firstforward.robot.modules.leds;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDModule implements Module, Telemeter {
    RevBlinkinLedDriver.BlinkinPattern INTAKE_OCCUPIED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    RevBlinkinLedDriver.BlinkinPattern IDLE = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

    // modules
    IntakeModule intake;

    //servos
    private final RevBlinkinLedDriver led;

    public LEDModule(HardwareMap hardwareMap, IntakeModule intake) {
        led = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        this.intake = intake;
    }

    public void update() {
        if (intake != null && intake.hasMineral()) {
            led.setPattern(INTAKE_OCCUPIED);
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
    public Iterable<String> getTelemetryData() {
        return null;
    }
}
