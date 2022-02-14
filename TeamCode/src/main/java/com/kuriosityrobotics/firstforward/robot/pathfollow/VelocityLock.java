package com.kuriosityrobotics.firstforward.robot.pathfollow;

public class VelocityLock {
    public final boolean targetVelocity;
    public final double velocity;
    public final boolean allowAccel;

    public VelocityLock(double velocity, boolean allowAccel) {
        this.targetVelocity = true;
        this.velocity = velocity;
        this.allowAccel = allowAccel;
    }

    public VelocityLock(double velocity) {
        this(velocity, false);
    }

    public VelocityLock() {
        this.targetVelocity = false;
        this.velocity = 0;
        this.allowAccel = false;
    }
}
