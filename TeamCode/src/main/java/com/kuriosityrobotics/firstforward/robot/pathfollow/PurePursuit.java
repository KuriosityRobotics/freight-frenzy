package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Circle;
import com.kuriosityrobotics.firstforward.robot.math.Line;
import com.kuriosityrobotics.firstforward.robot.math.Point;

import java.util.ArrayList;

public class PurePursuit {
    WayPoint[] path; // each pair of waypoints (e.g. 0 & 1, 1 & 2) is a segment of the path

    // params
    double followRadius;

    // helpers
    int pathIndex; // which path segment are we following?

    public PurePursuit(WayPoint[] path, double followRadius) {
        this.path = path;
        this.followRadius = followRadius;

        this.pathIndex = 0;
    }

    public void update(Robot robot, double moveSpeed) {
        Point robotPosition = robot.sensorThread.getOdometry().getPose();

        Point targetPosition = targetPosition(robotPosition);

        robot.drivetrain.setMovementsTowardPoint(targetPosition, moveSpeed);
    }

    private Point clipToPath(Point robotPosition) {
        double nearestClipDist = Double.MAX_VALUE;
        Point nearestClippedPoint = robotPosition;

        // Search the path we're on and the next one to see which one we're closer to
        // only search these two to make sure we don't skip any paths
        for (int i = pathIndex; i < Math.min(path.length - 1, pathIndex + 2); i++) {
            // path segment
            Line segment = new Line(path[i], path[i + 1]);

            Point clipped = robotPosition.projectToSegment(segment);
            double clipDistance = robotPosition.segmentDistance(segment);

            if (clipDistance < nearestClipDist) {
                nearestClipDist = clipDistance;
                nearestClippedPoint = clipped;
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
