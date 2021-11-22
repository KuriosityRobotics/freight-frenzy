package com.kuriosityrobotics.firstforward.robot.util.motionprofiling;

/**
 * One segment of a motion profile, e.g. a section with a constant acceleration (ramp up, maintian,
 * or ramp down).
 */
class MotionSegment {
    public double startVelocity, startDistanceAlongPath, endVelocity, endDistanceAlongPath;

    public MotionSegment(double startVelocity, double startDistanceAlongPath, double endVelocity, double endDistanceAlongPath) {
        this.startVelocity = startVelocity;
        this.startDistanceAlongPath = startDistanceAlongPath;

        this.endVelocity = endVelocity;
        this.endDistanceAlongPath = endDistanceAlongPath;
    }

    public double interpolateTargetVelocity(double distanceAlongPath) {
        if (distanceAlongPath < this.startDistanceAlongPath || distanceAlongPath > this.endDistanceAlongPath) {
            throw new IllegalArgumentException("Attempting to interpolate with a point not in this MotionSegment!");
        }

        if (this.startVelocity == this.endVelocity) {
            return this.endVelocity;
        }

        double distanceSinceStart = distanceAlongPath - this.startDistanceAlongPath;
        // vf^2 = vi^2 + 2ad, a = (vf^2 - vi^2) / (2d)
        double accel = (Math.pow(this.endVelocity, 2) - Math.pow(this.startVelocity, 2)) / (2 * (this.endDistanceAlongPath - this.startDistanceAlongPath));

        // vf^2 = vi^2 + 2ad.
        return Math.sqrt(Math.pow(this.startVelocity, 2) + (2 * accel * distanceSinceStart));
    }

    @Override
    public String toString() {
        return "MotionSegment{" +
                "startVelocity=" + startVelocity +
                ", startDistanceAlongPath=" + startDistanceAlongPath +
                ", endVelocity=" + endVelocity +
                ", endDistanceAlongPath=" + endDistanceAlongPath +
                '}';
    }
}