package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class IntakeAction extends Action {
    private static final long END_DELAY = 750;

    Long gotMineralTime;

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        robot.intakeModule.setIntakePower(-1);

        // if we've got the goods
        if (robot.intakeModule.hasMineral()) {
            this.gotMineralTime = SystemClock.elapsedRealtime();
        }

        this.completed = gotMineralTime != null && gotMineralTime + END_DELAY <= SystemClock.elapsedRealtime();
    }
}
