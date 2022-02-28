package com.kuriosityrobotics.firstforward.robot.util.PID;

public class FeedForwardPID {
    public final double f;

    private final ClassicalPID classicalPID;

    public FeedForwardPID(double f, double p, double i, double d) {
        this.classicalPID = new ClassicalPID(p, i, d);
        this.f = f;
    }

    public double calculateSpeed(double target, double error) {
        double scale = classicalPID.calculateSpeed(error);

        return (target * f) + (scale);
    }

    public void reset() {
        classicalPID.reset();
    }
}
