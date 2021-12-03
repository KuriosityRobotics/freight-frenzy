package com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling;

import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;

import java.util.ArrayList;

public class MotionProfile {
    public static final double ROBOT_MAX_VEL = 35;
    public static final double ROBOT_MAX_ACCEL = 90;
    public static final double ROBOT_MAX_DECCEL = 60;

    private double maxVel, maxAccel, maxDeccel;

    private ArrayList<MotionPathSegment> profile;

    public MotionProfile(WayPoint[] inputPath) {
        this(inputPath, ROBOT_MAX_VEL, ROBOT_MAX_ACCEL, ROBOT_MAX_DECCEL);
    }

    public MotionProfile(WayPoint[] inputPath, double maxVel, double maxAccel, double maxDeccel) {
        this.profile = new ArrayList<>();

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDeccel = maxDeccel;

        generateHeadingProfile(inputPath);
        generateVelocityProfile(inputPath);
    }

    private void generateVelocityProfile(WayPoint[] path) {
        // use the last specified velocity as the default velocity for unspecified points
        double lastVelocity = 0.1337;

        for (int i = 0; i < path.length - 1; i++) {
            double startVelo;
            double endVelo;

            if (i == 0) { // starting behavior
                if (path[i].hasTargetVelocity()) { // if we're given a starting velocity
                    startVelo = path[i].getVelocity();

                    // carry through starting velocity if end has no given target velo
                    endVelo = path[i + 1].hasTargetVelocity() ? path[i + 1].getVelocity() : startVelo;
                } else {
                    // default starting velo is something low
                    startVelo = 10;

                    // if second point doesn't have a target velo assume we're ramping up to max
                    endVelo = path[i + 1].hasTargetVelocity() ? path[i + 1].getVelocity() : this.maxVel;
                }
            } else { // 2nd + path segment
                startVelo = lastVelocity;

                // carry over starting velo if no next given
                endVelo = path[i + 1].hasTargetVelocity() ? path[i + 1].getVelocity() : startVelo;
            }

            // generate the segment
            MotionPathSegment segment = new MotionPathSegment(
                    new MotionPoint(path[i], startVelo, path[i].getAngleLock()),
                    new MotionPoint(path[i + 1], endVelo, path[i + 1].getAngleLock()),
                    endVelo >= startVelo ? maxAccel : maxDeccel
            );

            // use what value velocity actually gets to as the lastvelocity
            // due to constrained acceleration sometimes the velo we get to is not the velo targetted
            lastVelocity = segment.endVelocity();

            profile.add(segment);
        }
    }

    private WayPoint[] generateHeadingProfile(WayPoint[] in) {
        int interpolatedTo = 0;
        for (int i = 0; i < in.length; i++) {
            if (i < interpolatedTo) {
                continue;
            }

            WayPoint currentPoint = in[i];

            if (i == 0 && currentPoint.getAngleLock().getType() == AngleLock.AngleLockType.CONTINUE_LAST) {
//                throw new IllegalArgumentException("The first point in a path cannot have an angleLock of CONTINUE_LAST!");
                currentPoint.getAngleLock().type = AngleLock.AngleLockType.NO_LOCK;
            }

            if (currentPoint.getAngleLock().getType() == AngleLock.AngleLockType.LOCK) {
                if (i >= 2 && in[i - 1].getAngleLock().getType() == AngleLock.AngleLockType.NO_LOCK) {
                    double before = new Line(in[i - 2], in[i - 1]).getHeading();
                    double after = new Line(in[i-1], in[i]).getHeading();

                    in[i-1].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                    in[i-1].getAngleLock().heading = (before + after) / 2;
                }

                // Look ahead for the next LOCK or UNLOCK to interpolate the points in between
                boolean lockChanges = false;
                double totalDist = 0;
                int targetChangesIndex = 0;
                double nextTargetHeading = 0; // how much distance we have to change our heading

                // starting from the next point, look for where the next angle change must occur
                outerloop: //lemon
                for (int j = i + 1; j < in.length; j++) {
                    totalDist += in[j].distance(in[j - 1]);

                    // look for the next point that isn't CONTINUE_LAST
                    switch(in[j].getAngleLock().getType()) {
                        case LOCK:
                            // if it's LOCK, we're trying to get to that lock heading

                            lockChanges = true;
                            targetChangesIndex = j;
                            nextTargetHeading = in[j].getAngleLock().getHeading();

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
                        in[j].getAngleLock().heading = currentPoint.getAngleLock().getHeading();
                    }
                } else {
                    // interpolate rest of points so we get to the target heading
                    double distSoFar = 0;

                    double targetChange = nextTargetHeading - currentPoint.getAngleLock().getHeading();

                    for (int j = i + 1; j < targetChangesIndex; j++) {
                        distSoFar += in[j].distance(in[j - 1]);

                        double targetHeading = (distSoFar / totalDist) * (targetChange) + currentPoint.getAngleLock().getHeading();

                        in[j].getAngleLock().type = AngleLock.AngleLockType.LOCK;
                        in[j].getAngleLock().heading = targetHeading;
                    }
                }

                // jump over this section we just interpolated
                interpolatedTo = targetChangesIndex;
            } else if (currentPoint.getAngleLock().getType() == AngleLock.AngleLockType.NO_LOCK) {
                // switch all the next CONTINUE points to reflect this one
                int j = i;
                while (j + 1 < in.length && in[j+1].getAngleLock().getType() == AngleLock.AngleLockType.CONTINUE_LAST) {
                    in[j+1].getAngleLock().type = AngleLock.AngleLockType.NO_LOCK;
                    j++;
                }

                interpolatedTo = j;
            }
        }

        return in;
    }

    public double interpolateTargetVelocity(int pathIndex, Point clippedPosition) {
        return profile.get(pathIndex).interpolateTargetVelocity(clippedPosition);
    }

    public AngleLock interpolateTargetHeading(int pathIndex, Point clippedPosition) {
        return profile.get(pathIndex).interpolateTargetHeading(clippedPosition);
    }
}