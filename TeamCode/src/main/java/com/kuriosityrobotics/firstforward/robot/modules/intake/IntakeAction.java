package com.kuriosityrobotics.firstforward.robot.modules.intake;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakeAction extends Action {
    private static final long END_DELAY = 0;

    Long gotMineralTime;

    private final IntakeModule intakeModule;

    public IntakeAction(IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
    }

    @Override
    public void tick() {
        super.tick();

        intakeModule.intakePower = 1;

        long currentTime = SystemClock.elapsedRealtime();

        // if we've got the goods
        if (intakeModule.hasMineral()) {
            this.gotMineralTime = currentTime;
        }

        this.completed = gotMineralTime != null && gotMineralTime + END_DELAY <= currentTime;
    }
}