package com.kuriosityrobotics.firstforward.robot.pathfollow;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.max;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.math.Circle;
import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;
import com.kuriosityrobotics.firstforward.robot.util.DashboardUtil;
import com.kuriosityrobotics.firstforward.robot.util.FeedForwardPID;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.HashMap;

public class PurePursuit implements Telemeter {
    private final Robot robot;

    // constants
    private static final double STOP_THRESHOLD = 3;
    private static final double ANGLE_THRESHOLD = Math.toRadians(2);

    private final WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    private final boolean backwards;
    private final double followRadius;

    private final ActionExecutor actionExecutor;

    // motion magic
    private final MotionProfile profile;
    private final FeedForwardPID yPID = new FeedForwardPID(0.020, 0.0165, 0, 0);
    private final FeedForwardPID xPID = new FeedForwardPID(0.022, 0.03, 0, 0);
    private final ClassicalPID headingPID = new ClassicalPID(0.91, 0.000085, 0.09);
//    private final ClassicalPID headingPID = new ClassicalPID(0.85, 0, 0);

    // helpers
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.
    private int closestIndex; // which path segment is our robot closest to? updated in clipToPath()
    private boolean executedLastAction;

    double xvel, yvel, targx, targy, heading, targhead, targvel, vel;

    public PurePursuit(Robot robot, WayPoint[] path, boolean backwards, double followRadius) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);

        this.path = path;
        this.followRadius = followRadius;

        this.profile = new MotionProfile(path);

        this.actionExecutor = new ActionExecutor(robot);

        this.backwards = backwards;
        this.executedLastAction = false;
        this.pathIndex = 0;
        this.actionExecutor.execute(path[0]);
    }

    public PurePursuit(Robot robot, WayPoint[] path, double followRadius) {
        this(robot, path, false, followRadius);
    }

    public void follow() {
        while (robot.isOpModeActive()) {
            boolean atEnd = atEnd();
            if (atEnd && !executedLastAction) {
                actionExecutor.execute(path[path.length - 1]);
                executedLastAction = true;
            } else if (atEnd && executedLastAction && actionExecutor.doneExecuting()) {
                robot.drivetrain.setMovements(0, 0, 0);

                robot.telemetryDump.removeTelemeter(this);

                break;
            }

            this.update();
        }
    }

    public void update() {
        Pose robotPose = robot.sensorThread.getPose();
        Point robotVelo = robot.sensorThread.getVelocity();

        Point target = targetPosition(robotPose);

        Point clipped = clipToPath(robotPose);

        double targetVelocity = profile.interpolateTargetVelocity(closestIndex, clipped);
        AngleLock targetAngleLock = profile.interpolateTargetHeading(closestIndex, clipped);

        double headingToPoint = robot.drivetrain.relativeHeadingToPoint(target);
        double targetXVelo = targetVelocity * Math.sin(headingToPoint);
        double targetYVelo = targetVelocity * Math.cos(headingToPoint);

        double veloMag = Math.sqrt(Math.pow(robotVelo.x, 2) + Math.pow(robotVelo.y, 2));
        double veloHeading = Math.atan2(robotVelo.x, robotVelo.y);
        double alpha = veloHeading - robotPose.heading;
        double currXVelo = veloMag * Math.sin(alpha);
        double currYVelo = veloMag * Math.cos(alpha);

        double xPow = Range.clip(xPID.calculateSpeed(targetXVelo, (targetXVelo - currXVelo)), -1, 1);
        double yPow = Range.clip(yPID.calculateSpeed(targetYVelo, (targetYVelo - currYVelo)), -1, 1);
        double angPow;
        if (targetAngleLock.getType() == AngleLock.AngleLockType.NO_LOCK && robot.drivetrain.distanceToPoint(path[path.length - 1]) < STOP_THRESHOLD) {
            angPow = 0;
        } else {
            double targHeading;
            if (targetAngleLock.getType() == AngleLock.AngleLockType.NO_LOCK) {
                targHeading = this.backwards ? angleWrap(headingToPoint + Math.PI) : headingToPoint;
            } else {
                targHeading = targetAngleLock.getHeading();
            }
            targHeading = angleWrap(targHeading, Math.PI);
            double currHeading = angleWrap(robotPose.heading, Math.PI);

            double clockError = angleWrap(targHeading - currHeading, Math.PI);
            double counterError = angleWrap(currHeading - targHeading, Math.PI);
            double error = counterError > clockError ? clockError : -counterError;

            angPow = Range.clip(headingPID.calculateSpeed(error), -1, 1);
        }

        double normPow = xPow + yPow;
        double leftOver = 1 - angPow;
        double scale = normPow > leftOver ? (leftOver / normPow) : 1;
        xPow *= scale;
        yPow *= scale;

//        if (targetAngleLock.getType() == AngleLock.AngleLockType.NO_LOCK) {
//            angPow *= 0.6; // idk? it's less important??
//        }

        robot.drivetrain.setMovements(xPow, yPow, angPow);

        actionExecutor.tick();

        // save values for dashboard lemon
        vel = Math.sqrt(Math.pow(robotVelo.x, 2) + Math.pow(robotVelo.y, 2));
        targvel = targetVelocity;
        xvel = robotVelo.x;
        yvel = robotVelo.y;
        targx = targetXVelo;
        targy = targetYVelo;
        heading = robotPose.heading;
        targhead = targetAngleLock.getHeading();
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
        WayPoint end = path[path.length - 1];
        AngleLock lastAngle = end.getAngleLock();
        boolean angleEnd = lastAngle.getType() != AngleLock.AngleLockType.LOCK
                || (Math.abs(angleWrap(robot.drivetrain.getCurrentPose().heading - lastAngle.getHeading())) <= ANGLE_THRESHOLD && robot.drivetrain.getVelocity().heading < Math.toRadians(2));
        boolean stopped = !end.targetVelocity || end.velocity != 0 || (robot.drivetrain.getOrthVelocity() <= 3);
        return robot.drivetrain.distanceToPoint(path[path.length - 1]) <= STOP_THRESHOLD
                && angleEnd
                && stopped;
    }

    @Override
    public String getName() {
        return "PurePursuit";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Target heading: " + targhead);
        data.add("Target velocity: " + targvel);
        data.add("distToEnd: " + robot.drivetrain.distanceToPoint(path[path.length - 1]));
        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> map = new HashMap<>();

        map.put("xVel", "" + xvel);
        map.put("targx", "" + targx);
        map.put("yvel", "" + yvel);
        map.put("targy", "" + targy);
        map.put("Heading", "" + heading);
        map.put("targ heading", "" + targhead);
        map.put("targvel", "" + targvel);
        map.put("vel", "" + vel);

        return map;
    }
}
