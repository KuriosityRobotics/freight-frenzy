package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;

public abstract class Action {
    protected boolean completed;

    private boolean isStarted = false;
    private long startTime;

    /**
     * Executes the action, blocking until the action is completed.
     */
    public void execute(Robot robot) {
        while (!this.isCompleted()) {
            this.tick(robot);
        }
    }

    /**
     * Performs one tick of this action. Best for asynchronous execution.
     */
    public void tick(Robot robot) {
        if (!isStarted) {
            isStarted = true;
            startTime = SystemClock.elapsedRealtime();
        }
    }

    public boolean isCompleted() {
        return completed;
    }

    public long msSinceStart() {
        long currentTime = SystemClock.elapsedRealtime();
        return currentTime - startTime;
    }
}
