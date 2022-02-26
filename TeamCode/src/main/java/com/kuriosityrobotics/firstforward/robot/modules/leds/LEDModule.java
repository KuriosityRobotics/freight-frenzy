package com.kuriosityrobotics.firstforward.robot.modules.leds;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class LEDModule implements Module, Telemeter {
    // states
    public RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

    //servos
    private final RevBlinkinLedDriver LED;

    public LEDModule(HardwareMap hardwareMap) {
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void update() {
        this.LED.setPattern(pattern);
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
        return new ArrayList<>() {{
            add("Color: " + pattern.toString());
        }};
    }
}
