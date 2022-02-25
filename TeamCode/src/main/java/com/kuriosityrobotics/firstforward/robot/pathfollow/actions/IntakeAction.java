package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.modules.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakeAction extends Action {
    private static final long END_DELAY = 750;
    private final IntakeModule intakeModule;

    boolean startedRetracting = false;
    long startRetractionTime;

    public IntakeAction(IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
    }

    @Override
    public void tick() {
        super.tick();

        long currentTime = SystemClock.elapsedRealtime();
        if (startedRetracting && currentTime > startRetractionTime + END_DELAY) {
            intakeModule.intakePower = 0;
            this.completed = true;
        } else {
            if (intakeModule.inRetractionState()) {
                startedRetracting = true;
                startRetractionTime = currentTime;
            }

            intakeModule.intakePower = 1;
        }
    }
}
