package com.kuriosityrobotics.firstforward.robot.modules.intake;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakePowerAction extends Action {
    IntakeModule intakeModule;
    double power;

    public IntakePowerAction(IntakeModule intakeModule, double power) {
        this.intakeModule = intakeModule;
        this.power = power;
    }

    @Override
    public void tick() {
        super.tick();

        intakeModule.intakePower = this.power;

        this.completed = true;
    }
}
