package com.kuriosityrobotics.firstforward.robot.modules;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class LEDModule implements Module, Telemeter {
    public enum Color {
        RED(0.61),
        GREEN(0.7149999),
        WHITE(0.7569),
        OFF(0.778);

        private final double pwm;

        Color(double pwm) {
            this.pwm = pwm;
        }
    }

    private final Robot robot;

    // states
    public LEDModule.Color color;

    //servos
    private final Servo LED;

    public LEDModule(Robot robot) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);

        LED = robot.getServo("LED");
    }

    public void update() {
        this.LED.setPosition(color.pwm);
    }

    public LEDModule.Color getCurrentColor() {
        return this.color;
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
            add("Color: " + color.toString());
        }};
    }
}
