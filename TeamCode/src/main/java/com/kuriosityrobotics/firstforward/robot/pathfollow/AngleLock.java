package com.kuriosityrobotics.firstforward.robot.pathfollow;

public class AngleLock {
    public enum AngleLockType {
        CONTINUE_LAST, LOCK, NO_LOCK
    }

    public AngleLockType type;
    public double heading;

    public AngleLock(AngleLockType type, double heading) {
        this.type = type;
        this.heading = heading;
    }

    public AngleLock() {
        this(AngleLockType.CONTINUE_LAST, 0);
    }

    public AngleLock(double heading) {
        this(AngleLockType.LOCK, heading);
    }

    public AngleLockType getType() {
        return type;
    }

    public double getHeading() {
        return heading;
    }

    @Override
    public String toString() {
        return "type=" + type +
                ", heading=" + heading;
    }
}
