package com.kuriosityrobotics.firstforward.robot.pathfollow;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class PauseAction extends Action {
    private static final long WAIT_TIME_MS = 2000;

    @Override
    public void tick() {
        super.tick();

        if (this.msSinceStart() >= WAIT_TIME_MS) {
            this.completed = true;
        }
    }
}
