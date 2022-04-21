package com.kuriosityrobotics.firstforward.robot.pathfollow;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.ModuleThread;
import com.kuriosityrobotics.firstforward.robot.util.PID.IThresholdPID;
import com.kuriosityrobotics.firstforward.robot.util.math.Circle;
import com.kuriosityrobotics.firstforward.robot.util.math.Line;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.drivetrain.Drivetrain;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.PID.FeedForwardPID;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PurePursuit implements Telemeter {
    // constants
    private static final double STOP_THRESHOLD = 3;
    private static final double ANGLE_THRESHOLD = Math.toRadians(7);

    private final WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    private final boolean backwards;
    private final double followRadius;

    // motion magic
    private final MotionProfile profile;
    // avoid using I for x&y so we don't get funky behavior when we prioritize turning and fall behind on x+y
//    private final FeedForwardPID yPID = new FeedForwardPID(0.021, 0.007, 0, 0.00);
//    private final FeedForwardPID xPID = new FeedForwardPID(0.058, 0.027, 0.0000, 0);
    private final FeedForwardPID yPID = new FeedForwardPID(0.0185, 0.015, 0, 0.00);
    private final FeedForwardPID xPID = new FeedForwardPID(0.027, 0.026, 0.0000, 0);
    private final IThresholdPID headingPID = new IThresholdPID(0.68, 0.00035, 0.10, Math.toRadians(4), Math.toRadians(12));
    //    private final ClassicalPID headingPID = new ClassicalPID(0.67, 0.000, 0.10);
    double targetXVel, targetYVel, targx, targy, heading, targhead, targvel, vel, distToEnd;

    Point target = new Point(0, 0);
    // helpers
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.
    private int closestIndex; // which path segment is our robot closest to? updated in clipToPath()
    private boolean executedLastAction;
    private boolean pathEnding;
    private boolean started = false;
    private final double angleThreshold;
    public boolean fuzzyLastAction = false;

    public final double KILL_SWITCH_MS = 5000;

    public PurePursuit(WayPoint[] path, boolean backwards, double followRadius, double maxVel, double maxAccel, double maxDeccel) {
        this.path = path;
        this.followRadius = followRadius;

        this.profile = new MotionProfile(path, maxVel, maxAccel, maxDeccel);

        this.backwards = backwards;
        this.angleThreshold = ANGLE_THRESHOLD;

        this.reset();
    }

    public PurePursuit(WayPoint[] path, boolean backwards, double followRadius, double maxAccel, double maxDeccel) {
        this(path, backwards, followRadius, MotionProfile.ROBOT_MAX_VEL, maxAccel, maxDeccel);
    }

    public PurePursuit(WayPoint[] path, boolean backwards, double followRadius, double angleThreshold) {
        this.path = path;
        this.followRadius = followRadius;

        this.profile = new MotionProfile(path);

        this.backwards = backwards;
        this.angleThreshold = angleThreshold;

        this.reset();
    }

    public PurePursuit(WayPoint[] path, boolean backwards, double followRadius) {
        this(path, backwards, followRadius, ANGLE_THRESHOLD);
    }

    public PurePursuit(WayPoint[] path, double followRadius, double angleThreshold) {
        this(path, false, followRadius, angleThreshold);
    }

    public PurePursuit(WayPoint[] path, double followRadius) {
        this(path, false, followRadius);
    }

    private long pathStartTime = 0;

    public void reset() {
        this.pathIndex = 0;
        this.executedLastAction = false;
        this.started = false;

        xPID.reset();
        yPID.reset();
        headingPID.reset();

        for (WayPoint point : path) {
            point.resetActions();
        }
    }


    /**
     * Tick pure pursuit once. Returns true if this should be called again. Returns false if the
     * path is done.
     */
    public boolean update(LocationProvider locationProvider, Drivetrain drivetrain) {
        if (!started) {
            ActionExecutor.execute(path[0]);
            pathStartTime = SystemClock.elapsedRealtime();
            started = true;
        }

        if (SystemClock.elapsedRealtime() - pathStartTime > KILL_SWITCH_MS) {
            Log.e("PP", "gve up on path;  fix ur shit it broke");
            ModuleThread.KILL = true;
        }


        boolean atEnd = atEnd(locationProvider);
        if ((atEnd || (fuzzyLastAction && locationProvider.getPose().distance(path[path.length - 1]) < 1.75)) && !executedLastAction) {
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

        Point clipped = clipToPath(robotPose);

        Point target = targetPosition(clipped);

        Log.v("PP", "curr: " + robotPose);
        Log.v("PP", "targ: " + target);

        double targetVelocity = profile.interpolateTargetVelocity(closestIndex, clipped);
        AngleLock targetAngleLock = profile.interpolateTargetAngleLock(closestIndex, clipped);

        double headingToPoint = robotPose.relativeHeadingToPoint(target);
        double targetXVelo = targetVelocity * Math.sin(headingToPoint);
        double targetYVelo = targetVelocity * Math.cos(headingToPoint);

        Log.v("PP", "targX: " + targetXVelo + " targY: " + targetYVelo);

        double veloMag = Math.hypot(robotVelo.x, robotVelo.y);
        double veloHeading = Math.atan2(robotVelo.x, robotVelo.y);
        double alpha = veloHeading - robotPose.heading;
        double currXVelo = veloMag * Math.sin(alpha);
        double currYVelo = veloMag * Math.cos(alpha);

        Log.v("PP", "currx: " + currXVelo + "curry: " + currYVelo);

        Log.v("PP", "targangle: " + targetAngleLock);

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
            double error = angleWrap(targHeading - robotPose.heading);
            Log.v("PP", "ang error: " + error);

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
        targetXVel = currXVelo;
        targetYVel = currYVelo;
        targx = targetXVelo;
        targy = targetYVelo;
        heading = angleWrap(robotPose.heading, Math.PI);
        targhead = angleWrap(targetAngleLock.heading, Math.PI);
        distToEnd = locationProvider.distanceToPoint(path[path.length - 1]);

        this.target = target;
    }

    private Point clipToPath(Point robotPosition) {
        double nearestClipDist = Double.MAX_VALUE;
        Point nearestClippedPoint = path[0]; // default clip to start of path?

        // starting from the segment we're following and going backwards, find the segment that the robot is closest to.
        // clip the robot's position onto that segment.
        for (int i = Math.min(path.length - 2, pathIndex); i >= pathIndex; i--) {
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
//        return path[path.length - 1];
        return path[pathIndex + 1];
    }

    public boolean atEnd(LocationProvider locationProvider) {
        WayPoint end = path[path.length - 1];
        AngleLock lastAngle = profile.getLastAngleLock();

        // TODO fix this monkey angle thresholding
        boolean angVeloEnd = end.velocityLock.velocity != 0 || Math.abs(locationProvider.getVelocity().heading) < Math.toRadians(1.5);
        boolean angleEnd = lastAngle.type != AngleLock.AngleLockType.LOCK
                || (Math.abs(angleWrap(locationProvider.getPose().heading - lastAngle.heading)) <= ANGLE_THRESHOLD && angVeloEnd);
        boolean stopped = !end.getVelocityLock().targetVelocity || end.velocityLock.velocity != 0 || (locationProvider.getOrthVelocity() <= 1);

//        Log.v("PP", "angleend: " + angleEnd + " angerr: " + (Math.abs(angleWrap(locationProvider.getPose().heading - lastAngle.heading))) + " stopped: " + stopped);
//        Log.v("PP", "angvelo: " + locationProvider.getVelocity().heading);

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
    public List<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("target point: " + ((Point) target).toString());
        data.add("Target heading: " + targhead);
        data.add("Target velocity: " + targvel);
        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> map = new HashMap<>();

        map.put("xVel", "" + targetXVel);
        map.put("targx", "" + targx);
        map.put("yvel", "" + targetYVel);
        map.put("targy", "" + targy);
        map.put("Heading", "" + heading);
        map.put("targ heading", "" + targhead);
        map.put("targvel", "" + targvel);
        map.put("vel", "" + vel);

        return map;
    }
}
