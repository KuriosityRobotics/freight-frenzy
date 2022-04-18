package com.kuriosityrobotics.firstforward.robot.util;

import android.os.SystemClock;

public class Timer {
    private final long millis;
    private final Runnable onComplete;

    private long alarmTime;
    private boolean done = false;

    private Timer(long millis, Runnable onComplete) {
        this.alarmTime = SystemClock.elapsedRealtime() + millis;
        this.onComplete = onComplete;
        this.millis = millis;
    }

    public static Timer doIn(long millis, Runnable onComplete) {
        return new Timer(millis, onComplete);
    }

    public static Timer alarmIn(long millis) {
        return new Timer(millis, () -> {
        });
    }

    public synchronized boolean tick() {
        if (done)
            return true;
        else if (SystemClock.elapsedRealtime() > alarmTime) {
            onComplete.run();
            return done = true;
        } else
            return false;
    }

    public synchronized void reset() {
        this.alarmTime = SystemClock.elapsedRealtime() + millis;
        done = false;
    }

    public synchronized long millisElapsedSinceStart() {
        return SystemClock.elapsedRealtime() - (alarmTime - millis);
    }
}