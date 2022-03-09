package com.kuriosityrobotics.firstforward.robot.pathfollow;

import androidx.annotation.NonNull;

public class VelocityLock {
    public int pathIndex = -1;
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

    @NonNull
    @Override
    public String toString() {
        return "VelocityLock{" +
                "targetVelocity=" + targetVelocity +
                ", velocity=" + velocity +
                ", allowAccel=" + allowAccel +
                '}';
    }
}
