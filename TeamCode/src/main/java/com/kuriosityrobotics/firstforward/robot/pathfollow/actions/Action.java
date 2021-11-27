package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import android.os.SystemClock;

public abstract class Action {
    private boolean completed;

    private boolean isStarted = false;
    private long startTime;
    private long timeSinceStart;

    public void tick() {
        long currentTime = SystemClock.elapsedRealtime();
        if (!isStarted) {
            isStarted = true;
            startTime = currentTime;
        }
        timeSinceStart = currentTime - startTime;
    }

    public boolean isCompleted() {
        return completed;
    }
}