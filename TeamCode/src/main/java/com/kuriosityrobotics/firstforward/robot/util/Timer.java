package com.kuriosityrobotics.firstforward.robot.util;

import android.os.SystemClock;

public class Timer {
    private final long alarmTime;
    private final Runnable onComplete;
    private boolean done = false;

    public boolean tick() {
        if (done)
            return true;
        else if (SystemClock.elapsedRealtime() > alarmTime) {
            onComplete.run();
            return done = true;
        } else
            return false;
    }

    private Timer(long millis, Runnable onComplete) {
        this.alarmTime = SystemClock.elapsedRealtime() + millis;
        this.onComplete = onComplete;
    }

    public static Timer doIn(long millis, Runnable onComplete) {
        return new Timer(millis, onComplete);
    }
}
