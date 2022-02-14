package com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling;

import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.ListIterator;
import java.util.Map;

public class MotionProfile {
    public static final double ROBOT_MAX_VEL = 35;
    public static final double ROBOT_MAX_ACCEL = 90;
    public static final double ROBOT_MAX_DECCEL = 45;

    private double maxVel, maxAccel, maxDeccel;

    private WayPoint[] path;
    private WayPoint[] angleProfile;
    private ArrayList<MotionSegment> velocityProfile;

    public MotionProfile(WayPoint[] inputPath) {
        this(inputPath, ROBOT_MAX_VEL, ROBOT_MAX_ACCEL, ROBOT_MAX_DECCEL);
    }

    public MotionProfile(WayPoint[] inputPath, double maxVel, double maxAccel, double maxDeccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDeccel = maxDeccel;

        this.path = inputPath;

        this.angleProfile = interpolateHeadingProfile(inputPath);
        this.velocityProfile = generateVelocityProfile(inputPath);
    }

    private WayPoint[] interpolateHeadingProfile(WayPoint[] in) {
        int interpolatedTo = 0;
        for (int i = 0; i < in.length; i++) {
            if (i < interpolatedTo) {
                continue;
            }

            WayPoint currentPoint = in[i];

            if (i == 0 && currentPoint.getAngleLock().type == AngleLock.AngleLockType.CONTINUE_LAST) {
//                throw new IllegalArgumentException("The first point in a path cannot have an angleLock of CONTINUE_LAST!");
                currentPoint.getAngleLock().type = AngleLock.AngleLockType.NO_LOCK;
            }

            if (currentPoint.getAngleLock().type == AngleLock.AngleLockType.LOCK) {
                if (i >= 2 && in[i - 1].getAngleLock().type == AngleLock.AngleLockType.NO_LOCK) {
                    double before = new Line(in[i - 2], in[i - 1]).getHeading();
                    double after = new Line(in[i - 1], in[i]).getHeading();

                    in[i - 1].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                    in[i - 1].getAngleLock().heading = (before + after) / 2;
                }

                // Look ahead for the next LOCK or UNLOCK to interpolate the points in between
                boolean lockChanges = false;
                double totalDist = 0;
                int targetChangesIndex = 0;
                double nextTargetHeading = 0; // how much distance we have to change our heading

                // starting from the next point, look for where the next angle change must occur
                outerloop:
                //lemon
                for (int j = i + 1; j < in.length; j++) {
                    totalDist += in[j].distance(in[j - 1]);

                    // look for the next point that isn't CONTINUE_LAST
                    switch (in[j].getAngleLock().type) {
                        case LOCK:
                            // if it's LOCK, we're trying to get to that lock heading

                            lockChanges = true;
                            targetChangesIndex = j;
                            nextTargetHeading = in[j].getAngleLock().heading;

                            break outerloop;
                        case NO_LOCK:
                            // if it's NO_LOCK, we'll estimate what angle we should be at at that point
                            // by averaging the angles of the path before and after :)

                            // this only has meaning if it isn't the last point on the path
                            if (j != in.length - 1) {
                                lockChanges = true;
                                targetChangesIndex = j;

                                double inHeading = new Line(in[j - 1], in[j]).getHeading();
                                double outHeading = new Line(in[j], in[j + 1]).getHeading();

                                nextTargetHeading = (inHeading + outHeading) / 2;
                                in[j].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                                in[j].getAngleLock().heading = nextTargetHeading;
                            }

                            break outerloop;
                    }
                }

                // if we just keep this heading locked forever
                if (!lockChanges) {
                    // update rest of points to lock at that heading
                    for (int j = i + 1; j < in.length; j++) {
                        in[j].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                        in[j].getAngleLock().heading = currentPoint.getAngleLock().heading;
                    }
                } else {
                    // interpolate rest of points so we get to the target heading
                    double distSoFar = 0;

                    double targetChange = nextTargetHeading - currentPoint.getAngleLock().heading;

                    for (int j = i + 1; j < targetChangesIndex; j++) {
                        distSoFar += in[j].distance(in[j - 1]);

                        double targetHeading = (distSoFar / totalDist) * (targetChange) + currentPoint.getAngleLock().heading;

                        in[j].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                        in[j].getAngleLock().heading = targetHeading;
                    }
                }

                // jump over this section we just interpolated
                interpolatedTo = targetChangesIndex;
            } else if (currentPoint.getAngleLock().type == AngleLock.AngleLockType.NO_LOCK) {
                // switch all the next CONTINUE points to reflect this one
                int j = i;
                while (j + 1 < in.length && in[j + 1].getAngleLock().type == AngleLock.AngleLockType.CONTINUE_LAST) {
                    in[j + 1].getAngleLock().type = AngleLock.AngleLockType.NO_LOCK;
                    j++;
                }

                interpolatedTo = j;
            }
        }

        return in;
    }

    private ArrayList<MotionSegment> generateVelocityProfile(WayPoint[] path) {
        LinkedHashMap<Double, VelocityLock> velocityCheckPoints = generateVelocityCheckpoints(path);
        ArrayList<MotionSegment> profile = new ArrayList<>();

        // start from the back and generate forwards to make sure we can end where we want to
        // this means our profile will be backwards, we'll flip at end
        // though technically currently order doesn't matter

        // there's a corner case for this logic currently we should fix later:
        // if there's three points, low high low, the current implementation lowers the second
        // point as needed to make sure we can deccel from high to low. But it doesn't guarantee
        // that it's adjusted low enough so that we can go from the first to the second point.
        // it's hard to say if this is really a corner case, because technically the second 'high'
        // point is programmed as part of the input path

        ListIterator<Map.Entry<Double, VelocityLock>> iterator = new ArrayList<>(velocityCheckPoints.entrySet()).listIterator(velocityCheckPoints.size());
        Map.Entry<Double, VelocityLock> lastCheckpoint = iterator.previous();
        while (iterator.hasPrevious()) {
            double nextDistAlongPath = lastCheckpoint.getKey();
            double nextVel = lastCheckpoint.getValue().velocity;

            Map.Entry<Double, VelocityLock> checkpoint = iterator.previous();
            double currentDist = checkpoint.getKey();
            double currentVel = checkpoint.getValue().velocity;

            double deltaDist = nextDistAlongPath - currentDist;

            if (currentVel == nextVel) {
                profile.add(new MotionSegment(currentVel, currentDist, nextVel, nextDistAlongPath));
            } else {
                double accel = (nextVel > currentVel) ? maxAccel : -maxDeccel;
                double distanceNeeded = (Math.pow(nextVel, 2) - Math.pow(currentVel, 2)) / (2 * accel);

                if (distanceNeeded >= deltaDist) { // we have to adjust the current target velo
                    // so the acceleration is actually possible

                    double actualVel = Math.sqrt(Math.pow(currentVel, 2) + (2 * accel * distanceNeeded));

                    profile.add(new MotionSegment(currentDist, actualVel,
                            nextDistAlongPath, nextVel));

                    checkpoint.setValue(new VelocityLock(actualVel, checkpoint.getValue().allowAccel));
                } else {
                    if (checkpoint.getValue().allowAccel) {
                        // let's try to go as high as we can while still starting and ending
                        // at the right velocities

                        double switchDist = (Math.pow(nextVel, 2) - Math.pow(currentVel, 2) + (2 * maxDeccel * deltaDist))
                                / ((2 * maxAccel) + (2 * maxDeccel));
                        double highestVel = Math.sqrt(Math.pow(currentVel, 2) + (2 * maxAccel * switchDist));

                        // if we're capable of getting to a velocity higher than our max,
                        // we'll just accelerate to the max vel and back down
                        if (highestVel > maxVel) {
                            double up = (Math.pow(maxVel, 2) - Math.pow(currentVel, 2)) / (2 * maxAccel);
                            double down = (Math.pow(nextVel, 2) - Math.pow(maxVel, 2)) / (2 * maxDeccel);

                            profile.add(new MotionSegment(maxVel, deltaDist - down,
                                    nextVel, nextDistAlongPath));
                            profile.add(new MotionSegment(maxVel, currentDist + up,
                                    maxVel, distanceNeeded - down));
                            profile.add(new MotionSegment(currentVel, currentDist,
                                    maxVel, currentDist + up));
                        } else {
                            profile.add(new MotionSegment(highestVel, currentDist + switchDist,
                                    nextVel, nextDistAlongPath));
                            profile.add(new MotionSegment(currentVel, currentDist,
                                    highestVel, currentDist + switchDist));
                        }
                    } else {
                        if (Math.signum(accel) < 0) { // if we're deccel, hold current velo and then deccel
                            // remember we have to add backwards
                            profile.add(new MotionSegment(currentVel, nextDistAlongPath - distanceNeeded,
                                    nextVel, nextDistAlongPath));
                            profile.add(new MotionSegment(currentVel, currentDist,
                                    currentVel, nextDistAlongPath - distanceNeeded));
                        } else { // if we're accel, accel asap and then hold
                            // remember we're adding backwards
                            profile.add(new MotionSegment(nextVel, currentDist + distanceNeeded,
                                    nextVel, nextDistAlongPath));
                            profile.add(new MotionSegment(currentVel, currentDist,
                                    nextVel, currentDist + distanceNeeded));
                        }
                    }
                }
            }

            lastCheckpoint = checkpoint;
        }

        Collections.reverse(profile);

        return profile;
    }

    private LinkedHashMap<Double, VelocityLock> generateVelocityCheckpoints(WayPoint[] path) {
        LinkedHashMap<Double, VelocityLock> velocityCheckPoints = new LinkedHashMap<>();

        double dist = 0;

        if (path[0].getVelocityLock().targetVelocity) {
            velocityCheckPoints.put(dist, path[0].getVelocityLock());
        } else {
            // starting velocity can't be 0 or else the robot will never start moving
            // can look into using lookahead instead
            velocityCheckPoints.put(0., new VelocityLock(5));
        }

        for (int i = 0; i < path.length; i++) {
            WayPoint start = path[i];
            WayPoint end = path[i + 1];

            dist += start.distance(end);

            if (end.getVelocityLock().targetVelocity) {
                velocityCheckPoints.put(dist, end.getVelocityLock());
            }
        }

        return velocityCheckPoints;
    }

    public double interpolateTargetVelocity(int pathIndex, Point clippedPosition) {
        double distAlongPath = distanceAlongPath(pathIndex, clippedPosition);
        for (MotionSegment segment : velocityProfile) {
            if (distAlongPath >= segment.startDistanceAlongPath && distAlongPath <= segment.endDistanceAlongPath) {
                return segment.interpolateTargetVelocity(distAlongPath);
            }
        }
        throw new Error("Trying to interpolate to a distance outside of generated profile!");
    }

    public AngleLock interpolateTargetHeading(int pathIndex, Point clippedPosition) {
        WayPoint start = angleProfile[pathIndex];
        WayPoint end = angleProfile[pathIndex + 1];

        if (end.getAngleLock().type == AngleLock.AngleLockType.NO_LOCK) {
            return end.getAngleLock();
        } else if (end.getAngleLock().type == AngleLock.AngleLockType.LOCK) {
            if (start.getAngleLock().type == AngleLock.AngleLockType.LOCK) {
                double startHeading = start.getAngleLock().heading;
                double endHeading = end.getAngleLock().heading;

                double distAlong = start.distance(clippedPosition);
                double totalDist = start.distance(end);

                if (distAlong > totalDist) {
                    throw new IllegalArgumentException("Clipped position isn't within profiled bounds!");
                }

                return new AngleLock(((distAlong / totalDist) * (endHeading - startHeading)) + startHeading);
            } else {
                return end.getAngleLock();
            }
        } else {
            throw new IllegalArgumentException("Each point of a MotionPathSegment should either be angle locked or unlocked! (e.g. no CONTINUE) This should be guaranteed by interpolatePath() in MotionProfile.");
        }
    }

    public double distanceAlongPath(int pathIndex, Point clippedPosition) {
        double dist = 0;
        // add up all the paths prior to the one we're on
        for (int i = 0; i < pathIndex - 1; i++) {
            WayPoint start = path[i];
            WayPoint end = path[i + 1];

            dist += start.distance(end);
        }

        dist += path[pathIndex].distance(clippedPosition);

        return dist;
    }
}