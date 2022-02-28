package com.kuriosityrobotics.firstforward.robot.modules.intake;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class StopIntakeAction extends Action {
    IntakeModule intakeModule;

    public StopIntakeAction(IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
    }

    @Override
    public void tick() {
        super.tick();

        intakeModule.intakePower = 0;

        this.completed = true;
    }
}
