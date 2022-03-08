package com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.collections4.OrderedMapIterator;
import org.apache.commons.collections4.map.LinkedMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ListIterator;
import java.util.Map;

public class MotionProfile {
    public static final double ROBOT_MAX_VEL = 65;
    public static final double ROBOT_MAX_ACCEL = 95;
    public static final double ROBOT_MAX_DECCEL = 45;

    private final double maxVel, maxAccel, maxDeccel;

    private final WayPoint[] path;
    private final LinkedMap<Double, AngleLock> angleLockProfile;
    private final ArrayList<MotionSegment> velocityProfile;

    public MotionProfile(WayPoint[] inputPath) {
        this(inputPath, ROBOT_MAX_VEL, ROBOT_MAX_ACCEL, ROBOT_MAX_DECCEL);
    }

    public MotionProfile(WayPoint[] inputPath, double maxVel, double maxAccel, double maxDeccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDeccel = maxDeccel;

        this.path = inputPath;

        this.angleLockProfile = generateAngleLockProfile(inputPath);
        this.velocityProfile = generateVelocityProfile(inputPath);
    }

    private LinkedMap<Double, AngleLock> generateAngleLockProfile(WayPoint[] in) {
        LinkedMap<Double, AngleLock> profile = new LinkedMap<>();

        double dist = 0;

        AngleLock lastLock;
        if (in[0].getAngleLock().type == AngleLock.AngleLockType.CONTINUE_LAST) {
            AngleLock lock = new AngleLock(AngleLock.AngleLockType.NO_LOCK, 0);
            profile.put(dist, lock);
            lastLock = lock;
        } else {
            profile.put(dist, in[0].getAngleLock());
            lastLock = in[0].getAngleLock();
        }
        for (int i = 1; i < in.length; i++) {
            WayPoint start = in[i - 1];
            WayPoint end = in[i];

            dist += start.distance(end);

            AngleLock lock = in[i].getAngleLock();

            switch (lock.type) {
                case CONTINUE_LAST:
                    // ensure there's an anglelock for the end of the path
                    if (i == in.length - 1) {
                        profile.put(dist, profile.get(profile.lastKey()));
                    }
                    continue;
                case NO_LOCK:
                    if (lastLock.type != AngleLock.AngleLockType.NO_LOCK) {
                        profile.put(dist, lock);
                    }
                    break;
                case LOCK:
                    profile.put(dist, lock);
                    break;
            }

            lastLock = lock;
        }

        return profile;
    }

    private ArrayList<MotionSegment> generateVelocityProfile(WayPoint[] path) {
        LinkedMap<Double, VelocityLock> velocityCheckPoints = generateVelocityCheckpoints(path);
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
                            double down = (Math.pow(nextVel, 2) - Math.pow(maxVel, 2)) / (2 * -maxDeccel);

                            profile.add(new MotionSegment(maxVel, nextDistAlongPath - down,
                                    nextVel, nextDistAlongPath));
                            profile.add(new MotionSegment(maxVel, currentDist + up,
                                    maxVel, nextDistAlongPath - down));
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

    private LinkedMap<Double, VelocityLock> generateVelocityCheckpoints(WayPoint[] path) {
        LinkedMap<Double, VelocityLock> velocityCheckPoints = new LinkedMap<>();

        double dist = 0;

        if (path[0].getVelocityLock().targetVelocity) {
            velocityCheckPoints.put(dist, path[0].getVelocityLock());
        } else {
            // starting velocity can't be 0 or else the robot will never start moving
            // can look into using lookahead instead
            velocityCheckPoints.put(0., new VelocityLock(10, true));
        }

        for (int i = 0; i < path.length - 1; i++) {
            WayPoint start = path[i];
            WayPoint end = path[i + 1];

            dist += start.distance(end);

            if (end.getVelocityLock().targetVelocity) {
                velocityCheckPoints.put(dist, end.getVelocityLock());
            } else if (i == path.length - 2) {
                // if this is the last segment and there's no lock specified
                // assume we carry on the last lock given
                velocityCheckPoints.put(dist, velocityCheckPoints.get(velocityCheckPoints.lastKey()));
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

    public AngleLock interpolateTargetAngleLock(int pathIndex, Point clippedPosition) {
        // the last passed lock command
        double distAlongPath = distanceAlongPath(pathIndex, clippedPosition);

        // find the last passed profile and the next
        OrderedMapIterator<Double, AngleLock> i = angleLockProfile.mapIterator();
        double lastDist = i.next();
        AngleLock lastLock = i.getValue();
        while (i.hasNext()) {
            i.next();

            double dist = i.getKey();
            AngleLock nextLock = i.getValue();

            if (distAlongPath <= dist) {
                switch (nextLock.type) {
                    case NO_LOCK:
                        return nextLock;
                    case LOCK:
                        if (lastLock.type != AngleLock.AngleLockType.LOCK) {
                            return nextLock;
                        } else {
                            double totalDist = dist - lastDist;
                            double distAlong = Range.clip(distAlongPath - lastDist, 0, totalDist);

                            double clockError = angleWrap(nextLock.heading - lastLock.heading, Math.PI);
                            double counterError = angleWrap(lastLock.heading - nextLock.heading, Math.PI);

                            double error = counterError > clockError ? clockError : -counterError;

                            if (!i.hasPrevious()) {
                                return nextLock;
                            } else {
                                return new AngleLock(((distAlong / totalDist) * error) + lastLock.heading);
                            }
                        }
                    default:
                        throw new Error("Profiled angleLocks contain an angleLock of type that is not LOCK or NO_LOCK! This should be guaranteed by generateAngleLockProfile() in MotionProfile");
                }
            }
            lastLock = nextLock;
        }

        throw new Error("No angleLock was found in the profile!");
    }

    public AngleLock getLastAngleLock() {
        return angleLockProfile.get(angleLockProfile.lastKey());
    }

    public double distanceAlongPath(int pathIndex, Point clippedPosition) {
        double dist = 0;
        // add up all the paths prior to the one we're on
        for (int i = 0; i < pathIndex; i++) {
            WayPoint start = path[i];
            WayPoint end = path[i + 1];

            dist += start.distance(end);
        }

        // sketchy
        // relies on path index always being less than the last point which ig is fine
        dist += Range.clip(path[pathIndex].distance(clippedPosition), 0, path[pathIndex].distance(path[pathIndex + 1]));

        return dist;
    }
}