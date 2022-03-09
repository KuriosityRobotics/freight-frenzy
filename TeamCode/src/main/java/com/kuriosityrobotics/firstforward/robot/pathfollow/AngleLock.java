package com.kuriosityrobotics.firstforward.robot.pathfollow;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import static java.lang.Double.NaN;

import androidx.annotation.NonNull;

public class AngleLock {
    public enum AngleLockType {
        CONTINUE_LAST, LOCK, NO_LOCK
    }

    public final AngleLockType type;
    public final double heading;
    public double error = NaN;

    public AngleLock(AngleLockType type, double heading) {
        this.type = type;
        this.heading = angleWrap(heading, Math.PI);
    }

    public AngleLock() {
        this(AngleLockType.CONTINUE_LAST, 0);
    }

    public AngleLock(double heading) {
        this(AngleLockType.LOCK, heading);
    }

    @NonNull
    @Override
    public String toString() {
        return "type: " + type + ", heading: " + heading;
    }
}
