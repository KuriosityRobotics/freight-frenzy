package com.kuriosityrobotics.firstforward.robot.math;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;

import androidx.annotation.NonNull;

/**
 * Used to define a point in space. Has two field variables, x and y to define a point (x,y) on an
 * euclidean plane. Empty constructor creates a point with value (0, 0).
 */
public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distance(Point other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public Point projectToLine(Line line) {
        Point nearest;
        if (line.isVertical()) {
            nearest = new Point(line.startPoint.x, y);
        } else if (line.getSlope() == 0) {
            nearest = new Point(x, line.startPoint.y);
        } else {
            Line perpendicular = new Line(this, -1 / line.getSlope());
            nearest = line.getIntersection(perpendicular);
        }

        return nearest;
    }

    //SEGMENTS
    public Point projectToSegment(Line line) {
        Point nearest = projectToLine(line);

        if (line.containsPoint(nearest)) {
            return nearest;
        } else {
            if (nearest.distance(line.startPoint) <= nearest.distance(line.endPoint)) {
                return line.startPoint;
            }
            return line.endPoint;
        }
    }

    //SEGMENTS
    public double segmentDistance(Line line) {
        return this.distance(projectToSegment(line));
    }

    public Point scale(double scale) {
        return new Point(scale * x, scale * y);
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Point)) {
            return false;
        }

        Point point = (Point) o;
        return doublesEqual(point.x, x) && doublesEqual(point.y, y);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }
}

