package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.os.SystemClock;

public abstract class Action {
    protected boolean completed = false;

    private boolean isStarted = false;
    private long startTime;

    /**
     * Executes the action, blocking until the action is completed.
     */
    public void execute() {
        while (!this.isCompleted()) {
            this.tick();
        }
    }

    /**
     * Performs one tick of this action. Best for asynchronous execution.
     */
    public void tick() {
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
