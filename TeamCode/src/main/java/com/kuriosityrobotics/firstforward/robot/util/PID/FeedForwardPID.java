package com.kuriosityrobotics.firstforward.robot.util.PID;

import android.os.SystemClock;

public class FeedForwardPID {
    public final double f;
    private long lastUpdateTime;

    private final ClassicalPID classicalPID;

    public FeedForwardPID(double f, double p, double i, double d) {
        this.classicalPID = new ClassicalPID(p, i, d);
        this.f = f;
        lastUpdateTime = NanoClock.now();
    }

    public double calculateSpeed(double target, double error) {
        error /= (NanoClock.now() - lastUpdateTime);
        double scale = classicalPID.calculateSpeed(error);

        lastUpdateTime = NanoClock.now();
        return (target * f) + (scale);
    }

    public void reset() {
        classicalPID.reset();
    }
}
