package com.kuriosityrobotics.firstforward.robot.modules;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.Servo;

import static com.kuriosityrobotics.firstforward.robot.modules.State.State;

import android.annotation.SuppressLint;

import java.util.ArrayList;

public class BlockerModule implements Module, Telemeter {
    private final StateMachine stateMachine;

    private final Servo leftBlockerServo;
    private final Servo flapServo;

    public BlockerModule(Robot robot) {

        robot.telemetryDump.registerTelemeter(this);
        stateMachine = new StateMachine();

        this.leftBlockerServo = robot.hardwareMap.get(Servo.class, "blockerLeft");
        this.flapServo = robot.hardwareMap.get(Servo.class, "shooterFlap");
    }

    public State OpenFlap() {
        return State(() -> {
            leftBlockerServo.setPosition(.4);
            flapServo.setPosition(0.29255);
        }, 0);
    }

    public State CloseFlap() {
        return State(() -> {
            leftBlockerServo.setPosition(.2);
            flapServo.setPosition(0.1766);
        }, 0);
    }

    public void setCurrentState(State state) {
        stateMachine.setCurrentState(state);
    }

    @Override
    public void update() {
        stateMachine.update();
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "BlockerModule";
    }

    @SuppressLint("DefaultLocale")
    @Override
    public Iterable<String> getTelemetryData() {
        var data = new ArrayList<String>();
        data.add(String.format("leftBlockerServo position:  %f, flapServo position:  %f", leftBlockerServo.getPosition(), flapServo.getPosition()));
        return data;
    }
}
