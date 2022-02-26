package com.kuriosityrobotics.firstforward.robot.pathfollow;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Circle;
import com.kuriosityrobotics.firstforward.robot.util.math.Line;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.drivetrain.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;
import com.kuriosityrobotics.firstforward.robot.util.FeedForwardPID;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.HashMap;

public class PurePursuit implements Telemeter {
    // constants
    private static final double STOP_THRESHOLD = 3;
    private static final double ANGLE_THRESHOLD = Math.toRadians(2);

    private final WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    private final boolean backwards;
    private final double followRadius;
    
    // motion magic
    private final MotionProfile profile;
    // avoid using I for x&y so we don't get funky behavior when we prioritize turning and fall behind on x+y
    private final FeedForwardPID yPID = new FeedForwardPID(0.0155, 0.0075, 0, 0.013);
    private final FeedForwardPID xPID = new FeedForwardPID(0.023, 0.018, 0, 0);
    private final ClassicalPID headingPID = new ClassicalPID(0.41, 0.000055, 0.21);
    double xvel, yvel, targx, targy, heading, targhead, targvel, vel, distToEnd;
    Point target = new Point(0, 0);
    // helpers
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.
    private int closestIndex; // which path segment is our robot closest to? updated in clipToPath()
    private boolean executedLastAction;
    private boolean pathEnding;

    public PurePursuit(WayPoint[] path, boolean backwards, double followRadius) {
        this.path = path;
        this.followRadius = followRadius;

        this.profile = new MotionProfile(path);

        this.backwards = backwards;
        this.executedLastAction = false;
        this.pathIndex = 0;
        ActionExecutor.execute(path[0]);
    }

    public PurePursuit(WayPoint[] path, double followRadius) {
        this(path, false, followRadius);
    }

    public boolean update(LocationProvider locationProvider, Drivetrain drivetrain) {
        boolean atEnd = atEnd(locationProvider);
        if (atEnd && !executedLastAction) {
            ActionExecutor.execute(path[path.length - 1]);
            executedLastAction = true;
        } else if (atEnd && executedLastAction && ActionExecutor.doneExecuting()) {
            drivetrain.setMovements(0, 0, 0);
            return false;
        }

        this.updateDrivetrain(locationProvider, drivetrain);
        ActionExecutor.tick();
        return true;
    }

    private void updateDrivetrain(LocationProvider locationProvider, Drivetrain drivetrain) {
        Pose robotPose = locationProvider.getPose();
        Point robotVelo = locationProvider.getVelocity();

        Point target = targetPosition(robotPose);

        Point clipped = clipToPath(robotPose);

        double targetVelocity = profile.interpolateTargetVelocity(closestIndex, clipped);
        AngleLock targetAngleLock = profile.interpolateTargetAngleLock(closestIndex, clipped);

        double headingToPoint = robotPose.relativeHeadingToPoint(target);
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

        pathEnding = locationProvider.distanceToPoint(path[path.length - 1]) < STOP_THRESHOLD || pathEnding;
        if (targetAngleLock.type == AngleLock.AngleLockType.NO_LOCK && pathEnding) {
            // if we overshoot the end point we don't want to turn back around to face it
            angPow = 0;
        } else {
            double targHeading;
            if (targetAngleLock.type == AngleLock.AngleLockType.NO_LOCK) {
                targHeading = this.backwards ? angleWrap(headingToPoint + Math.PI) : headingToPoint;
            } else {
                targHeading = targetAngleLock.heading;
            }
            targHeading = angleWrap(targHeading, Math.PI);
            double currHeading = angleWrap(robotPose.heading, Math.PI);

            double clockError = angleWrap(targHeading - currHeading, Math.PI);
            double counterError = angleWrap(currHeading - targHeading, Math.PI);
            double error = counterError > clockError ? clockError : -counterError;

            angPow = Range.clip(headingPID.calculateSpeed(error), -1, 1);
        }

        double normPow = Math.abs(xPow) + Math.abs(yPow);
        double leftOver = 1 - Math.abs(angPow);
        double scale = (normPow != 0 && normPow > leftOver) ? (leftOver / normPow) : 1;
        xPow *= scale;
        yPow *= scale;

//        if (targetAngleLock.getType() == AngleLock.AngleLockType.NO_LOCK) {
//            angPow *= 0.6; // idk? it's less important??
//        }

        drivetrain.setMovements(xPow, yPow, angPow);

        // save values for dashboard lemon
        vel = Math.sqrt(Math.pow(robotVelo.x, 2) + Math.pow(robotVelo.y, 2));
        targvel = targetVelocity;
        xvel = robotVelo.x;
        yvel = robotVelo.y;
        targx = targetXVelo;
        targy = targetYVelo;
        heading = robotPose.heading;
        targhead = targetAngleLock.heading;
        distToEnd = locationProvider.distanceToPoint(path[path.length - 1]);

        this.target = target;
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

            if (!pathIntersections.isEmpty()) {
                //returns point that is closer to the end of the segment

                if (i != pathIndex) {
                    for (int j = pathIndex + 1; j <= i; j++) {
                        WayPoint point = path[j];
                        ActionExecutor.execute(point);
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

    public boolean atEnd(LocationProvider locationProvider) {
        WayPoint end = path[path.length - 1];
        AngleLock lastAngle = profile.getLastAngleLock();
        boolean angleEnd = lastAngle.type != AngleLock.AngleLockType.LOCK
                || (Math.abs(angleWrap(locationProvider.getPose().heading - lastAngle.heading)) <= ANGLE_THRESHOLD && locationProvider.getVelocity().heading < Math.toRadians(1.5));
        boolean stopped = !end.getVelocityLock().targetVelocity || end.velocityLock.velocity != 0 || (locationProvider.getOrthVelocity() <= 3);
        return locationProvider.distanceToPoint(path[path.length - 1]) <= STOP_THRESHOLD
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
        data.add("target point: " + target.toString());
        data.add("Target heading: " + targhead);
        data.add("Target velocity: " + targvel);
        data.add("distToEnd: " + distToEnd);
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
