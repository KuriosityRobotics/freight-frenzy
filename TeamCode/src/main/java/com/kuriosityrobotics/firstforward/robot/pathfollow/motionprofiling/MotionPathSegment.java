package com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;

import java.util.ArrayList;

class MotionPathSegment {
    private ArrayList<MotionSegment> motionSegments;

    MotionPoint start;
    MotionPoint end;

    public MotionPathSegment(MotionPoint start, MotionPoint end) {
        this.start = start;
        this.end = end;

        this.motionSegments = new ArrayList<>();

        generateMotionProfile();
    }

    private void generateMotionProfile() {
        double totalDistance = start.distance(end);

        if (start.velocity == end.velocity) {
            // hold velo for the whole time
            motionSegments.add(new MotionSegment(start.velocity, 0, end.velocity, totalDistance));
        } else {
            double accelSign = Math.signum(end.velocity - start.velocity);

            // vf^2 = vi^2 + 2ad. d = ((vf^2 - vi^2) / 2a)
            double distanceNeeded = (Math.pow(end.velocity, 2) - Math.pow(start.velocity, 2)) / (2 * MotionProfile.MAX_ACCEL * accelSign);

            if (distanceNeeded > totalDistance) {
                // vf = sqrt(vi^2 + 2ad).
                double maxVel = Math.sqrt(Math.pow(start.velocity, 2) + (2 * accelSign * MotionProfile.MAX_ACCEL * distanceNeeded));

                // ramp up the entire way
                motionSegments.add(new MotionSegment(start.velocity, 0, maxVel, totalDistance));
            } else {
                if (end.velocity > start.velocity) {
                    // ramp up velo
                    motionSegments.add(new MotionSegment(start.velocity, 0, end.velocity, distanceNeeded));

                    // hold it for rest of path
                    motionSegments.add(new MotionSegment(end.velocity, distanceNeeded, end.velocity, totalDistance));
                } else {
                    // hold it
                    motionSegments.add(new MotionSegment(start.velocity, 0, start.velocity, totalDistance - distanceNeeded));

                    // ramp down velo
                    motionSegments.add(new MotionSegment(start.velocity, totalDistance - distanceNeeded, end.velocity, totalDistance));
                }
            }
        }
    }

    public double interpolateTargetVelocity(Point clippedPosition) {
        double distAlongPath = this.start.distance(clippedPosition);

        for (MotionSegment segment : this.motionSegments) {
            if (distAlongPath <= segment.endDistanceAlongPath && distAlongPath >= segment.startDistanceAlongPath) {
                return segment.interpolateTargetVelocity(distAlongPath);
            }
        }

        throw new IllegalArgumentException("Clipped position isn't within profiled bounds!");
    }

    public AngleLock interpolateTargetHeading(Point clippedPosition) {
        if (end.angleLock.getType() == AngleLock.AngleLockType.NO_LOCK) {
            return end.angleLock;
        } else if (end.angleLock.getType() == AngleLock.AngleLockType.LOCK) {
            if (start.angleLock.getType() == AngleLock.AngleLockType.LOCK) {
                double startHeading = start.angleLock.getHeading();
                double endHeading = end.angleLock.getHeading();

                double distAlong = start.distance(clippedPosition);
                double totalDist = start.distance(end);

                if (distAlong > totalDist) {
                    throw new IllegalArgumentException("Clipped position isn't within profiled bounds!");
                }

                return new AngleLock(((distAlong / totalDist) * (endHeading - startHeading)) + startHeading);
            } else {
                return start.angleLock;
            }
        } else {
            throw new IllegalArgumentException("Each point of a MotionPathSegment should either be angle locked or unlocked! (e.g. no CONTINUE) This should be guaranteed by interpolatePath() in MotionProfile.");
        }
    }

    public double endVelocity() {
        return motionSegments.get(motionSegments.size() - 1).endVelocity;
    }
}
