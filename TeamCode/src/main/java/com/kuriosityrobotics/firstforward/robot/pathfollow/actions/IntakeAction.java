package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakeAction extends Action {
    private static final long END_DELAY = 750;

    boolean startedRetracting = false;
    long startRetractionTime;

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        long currentTime = SystemClock.elapsedRealtime();
        if (startedRetracting && currentTime > startRetractionTime + END_DELAY) {
            robot.intakeModule.intakePower = 0;
            this.completed = true;
        } else {
            if (robot.intakeModule.isIntakeRetracting()) {
                startedRetracting = true;
                startRetractionTime = currentTime;
            }

            robot.intakeModule.intakePower = 1;
        }
    }
}
