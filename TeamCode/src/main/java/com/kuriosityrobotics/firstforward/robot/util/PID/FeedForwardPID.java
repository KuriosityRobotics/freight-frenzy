package com.kuriosityrobotics.firstforward.robot.util.PID;

import android.os.SystemClock;

public class FeedForwardPID {
    public final double f;
    private long lastUpdateTime;

    private final ClassicalPID classicalPID;

    public FeedForwardPID(double f, double p, double i, double d) {
        this.classicalPID = new ClassicalPID(p, i, d);
        this.f = f;
        lastUpdateTime = SystemClock.elapsedRealtimeNanos();
    }

    public double calculateSpeed(double target, double error) {
        error /= (SystemClock.elapsedRealtimeNanos() - lastUpdateTime);
        double scale = classicalPID.calculateSpeed(error);

        lastUpdateTime = SystemClock.elapsedRealtimeNanos();
        return (target * f) + (scale);
    }

    public void reset() {
        classicalPID.reset();
    }
}
