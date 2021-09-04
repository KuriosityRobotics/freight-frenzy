package com.kuriosityrobotics.firstforward.robot.modules;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;

public class FlapModule implements Module {
    private final Robot robot;

    StateMachine flap;

    private final Servo flapServo;

    public class OpenFlap implements StateMachine.State {
        @Override
        public void execute() {
            flapServo.setPosition(0.6);
        }

        @Override
        public long blockDuration() {
            return 1000;
        }
    }

    public class CloseFlap implements StateMachine.State {
        @Override
        public void execute() {
            flapServo.setPosition(0.5);
        }

        @Override
        public long blockDuration() {
            return 5000;
        }
    }

    public FlapModule(Robot robot) {
        this.robot = robot;

        flap = new StateMachine();

        this.flapServo = robot.hardwareMap.get(Servo.class, "flapServo");
    }

    @Override
    public void update() {
        flap.update();
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "FlapModule";
    }
}
