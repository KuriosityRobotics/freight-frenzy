package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Circle;
import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.util.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;
import com.kuriosityrobotics.firstforward.robot.util.FeedForwardPID;

import java.util.ArrayList;

public class PurePursuit {
    private final Robot robot;

    // constants
    private static final double STOP_THRESHOLD = 3;

    private WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    private double followRadius;

    private final ActionExecutor actionExecutor;

    // motion magic
    private final MotionProfile profile;
    private final FeedForwardPID yPID = new FeedForwardPID(0, 1, 0, 0);
    private final FeedForwardPID xPID = new FeedForwardPID(0, 1, 0, 0);
    private final ClassicalPID headingPID = new ClassicalPID(1, 0, 0);

    // helpers
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.
    private int closestIndex; // which path segment is our robot closest to? updated in clipToPath()
    private boolean executedLastAction;

    public PurePursuit(Robot robot, WayPoint[] path, double followRadius, double velocityFactor) {
        this.robot = robot;

        this.path = path;
        this.followRadius = followRadius;

        for (WayPoint point : path) {
            point.velocity *= velocityFactor;
        }

        this.profile = new MotionProfile(path);

        this.actionExecutor = new ActionExecutor(robot);

        this.executedLastAction = false;
        this.pathIndex = 0;
        this.actionExecutor.execute(path[0]);
    }

    public PurePursuit(Robot robot, WayPoint[] path, double followRadius) {
        this(robot, path, followRadius, 0);
    }

    public void follow() {
        while (robot.isOpModeActive()) {
            if (atEnd() && !executedLastAction) {
                actionExecutor.execute(path[path.length - 1]);
            } else if (atEnd() && actionExecutor.doneExecuting()) {
                robot.drivetrain.setMovements(0, 0, 0);
                return;
            }

            this.update();
        }
    }

    int i = 0;

    public void update() {
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
            Pose velo = robot.sensorThread.getOdometry().getVelocity();
            Log.v("PP", "velo: " + Math.sqrt((Math.pow(velo.x, 2) + Math.pow(velo.y, 2))));
            Log.v("PP", "Target velo: " + targetVelocity + " targetHeading: " + targetHeading);
            i = 0;
        }
        i++;

//        double x = xPID.calculateSpeed(targetXVelo, (targetXVelo - robotVelo.x));
//        double y = yPID.calculateSpeed(targetYVelo, (targetYVelo - robotVelo.y));
//        double heading = headingPID.calculateSpeed(targetHeading.getHeading() - robotPose.heading);
//
//        robot.drivetrain.setMovements(x, y, heading);

        actionExecutor.tick();
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
                    for (int j = pathIndex + 1; j <= i; j++) {
                        WayPoint point = path[j];
                        actionExecutor.execute(point);
                    }

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

    public boolean atEnd() {
        return robot.drivetrain.distanceToPoint(path[path.length - 1]) < STOP_THRESHOLD;
    }
}
