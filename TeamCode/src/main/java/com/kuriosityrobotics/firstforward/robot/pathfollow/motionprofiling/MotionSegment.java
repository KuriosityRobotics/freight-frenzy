package com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

/**
 * One segment of a motion profile, e.g. a section with a constant acceleration (ramp up, maintian,
 * or ramp down).
 */
class MotionSegment {
    public final double startVelocity;
    public final double startDistanceAlongPath;
    public final double endVelocity;
    public final double endDistanceAlongPath;

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

        double pathDist = (this.endDistanceAlongPath - this.startDistanceAlongPath);
        double distanceSinceStart = distanceAlongPath - this.startDistanceAlongPath;

        if (distanceSinceStart >= pathDist) {
            return endVelocity;
        }

        // vf^2 = vi^2 + 2ad, a = (vf^2 - vi^2) / (2d)
        double accel = (Math.pow(this.endVelocity, 2) - Math.pow(this.startVelocity, 2)) / (2 * pathDist);

        // vf^2 = vi^2 + 2ad.
        double vi2 = Math.pow(this.startVelocity, 2);
        double tad = (2 * accel * distanceSinceStart);

        // todo is there a less sketchy way to do this?
        // if we're trying to decel past 0, we get nasty NaNs from sqrt
        if (-tad > vi2) {
            return 0;
        }

        return Math.sqrt(Math.pow(this.startVelocity, 2) + (2 * accel * distanceSinceStart));
    }

    @NonNull
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