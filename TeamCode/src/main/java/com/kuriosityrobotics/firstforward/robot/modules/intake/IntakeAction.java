package com.kuriosityrobotics.firstforward.robot.modules.intake;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakeAction extends Action {
    private static final long END_DELAY = 750;

    Long gotMineralTime;

    private final IntakeModule intakeModule;

    public IntakeAction(IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
    }

    @Override
    public void tick() {
        super.tick();

        intakeModule.intakePower = 1;

        // if we've got the goods
        if (intakeModule.hasMineral()) {
            this.gotMineralTime = SystemClock.elapsedRealtime();
        }

        this.completed = gotMineralTime != null && gotMineralTime + END_DELAY <= SystemClock.elapsedRealtime();
    }
}