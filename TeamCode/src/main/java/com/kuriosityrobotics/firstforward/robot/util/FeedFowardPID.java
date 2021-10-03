package com.kuriosityrobotics.firstforward.robot.util;

public class FeedFowardPID {
    private final double f;

    private final ClassicalPID classicalPID;

    public FeedFowardPID(double f, double p, double i, double d) {
        this.classicalPID = new ClassicalPID(p, i, d);
        this.f = f;
    }

    public double calculateSpeed(double target, double error) {
        double scale = classicalPID.calculateSpeed(error);

        return (target * f) + (scale);
    }
}