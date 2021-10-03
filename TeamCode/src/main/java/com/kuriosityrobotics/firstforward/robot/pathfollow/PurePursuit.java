package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Circle;
import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;
import com.kuriosityrobotics.firstforward.robot.util.FeedFowardPID;

import java.util.ArrayList;

public class PurePursuit {
    WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    double followRadius;

    // motion magic
    private final MotionProfile profile;
    private final FeedFowardPID yPID = new FeedFowardPID(0, 1, 0, 0);
    private final FeedFowardPID xPID = new FeedFowardPID(0, 1, 0, 0);
    private final ClassicalPID headingPID = new ClassicalPID(1, 0, 0);

    // helpers
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.
    private int closestIndex; // which path segment is our robot closest to? updated in clipToPath()

    public PurePursuit(WayPoint[] path, double followRadius, double velocityFactor) {
        this.path = path;
        this.followRadius = followRadius;

        for (WayPoint point : path) {
            point.velocity *= velocityFactor;
        }

        this.pathIndex = 0;

        this.profile = new MotionProfile(path);
    }

    public PurePursuit(WayPoint[] path, double followRadius) {
        this(path, followRadius, 0);
    }

    int i = 0;
    public void update(Robot robot) {
        Pose robotPose = robot.sensorThread.getOdometry().getPose();
        Point robotVelo = robot.sensorThread.getOdometry().getVelocity();

        Point target = targetPosition(robotPose);

        Point clipped = clipToPath(robotPose);

        double targetVelocity = profile.interpolateTargetVelocity(closestIndex, clipped);
        AngleLock targetHeading = profile.interpolateTargetHeading(closestIndex, clipped);

        double headingToPoint = robot.drivetrain.relativeHeadingToPoint(target);
        double targetXVelo = targetVelocity * Math.sin(headingToPoint);
        double targetYVelo = targetVelocity * Math.cos(headingToPoint);

        robot.drivetrain.setMovementsTowardPoint(target, 0.8);

        if (i >= 500) {
//            Log.v("PP", "clipped: " + clipped + " closest: " + closestIndex);
            Log.v("PP", "Target velo: " + targetVelocity + " targetHeading: " + targetHeading);
            i = 0;
        }
        i++;
//        double x = xPID.calculateSpeed(targetXVelo, (targetXVelo - robotVelo.x));
//        double y = yPID.calculateSpeed(targetYVelo, (targetYVelo - robotVelo.y));
//        double heading = headingPID.calculateSpeed(targetHeading.getHeading() - robotPose.heading);
//
//        robot.drivetrain.setMovements(x, y, heading);
    }

    private Point clipToPath(Point robotPosition) {
        double nearestClipDist = Double.MAX_VALUE;
        Point nearestClippedPoint = path[0]; // default clip to start of path?

        // starting from the segment we're following and going backwards, find the segment that the robot is closest to.
        // clip the robot's position onto that segment.
        for (int i = Math.min(path.length - 2, pathIndex); i >= 0; i--) {
            Line segment = new Line(path[i], path[i + 1]);

            Point clipped = robotPosition.projectToSegment(segment);
            double clipDistance = robotPosition.distance(clipped);

            if (segment.containsPoint(clipped)) {
                if (clipDistance < nearestClipDist) {
                    nearestClippedPoint = clipped;
                    nearestClipDist = clipDistance;
                    closestIndex = i; // this is quite sus
                }
            }
        }

        // return the clipped point
        return nearestClippedPoint;
    }

    private Point targetPosition(Point center) {
        Circle radius = new Circle(center, this.followRadius);

        if (pathIndex == path.length - 2 && center.distance(path[path.length - 1]) < followRadius) {
            return path[path.length - 1];
        }

        // find the first path whose end is past the follow radius
        int lookAheadUntil = path.length - 2;
        for (int i = pathIndex; i < path.length - 2; i++) {
            Point endSegment = path[i + 1];

            if (center.distance(endSegment) > followRadius) {
                lookAheadUntil = i;
            }
        }
        // make sure we include the next path too. but don't go over the last path segment.
        lookAheadUntil = Math.min(path.length - 2, Math.max(lookAheadUntil, pathIndex + 1));

        // find intersections in this path or the next
        // search the furthest segment first (so the next one)
        // return the first intersection closest to the end of its path segment
        for (int i = lookAheadUntil; i >= pathIndex; i--) {
            Line pathSegment = new Line(path[i], path[i + 1]);

            ArrayList<Point> pathIntersections = radius.getSegmentIntersections(pathSegment);

            if (pathIntersections.size() == 0) {
                continue;
            } else {
                //returns point that is closer to the end of the segment
                if (i != pathIndex) {
                    // TODO do actions

                    // set the new target
                    pathIndex = i;
                }

                return pathSegment.closerToEnd(pathIntersections);
            }
        }

        // if we couldn't find any intersections
        // return the end of the path
        return path[path.length - 1];
    }
}
