package com.kuriosityrobotics.firstforward.robot.util.math;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import java.util.ArrayList;

public class Line {
    public Point startPoint;
    public Point endPoint;

    //for pathfollow (LINE SEGMENT)
    public Line(Point startPoint, Point endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
    }

    //for finding intersections (LINE)
    public Line(Point point, double slope) {
        this(point, new Point(point.x + 1, point.y + slope));
    }

    public boolean isVertical() {
        return doublesEqual(startPoint.x, endPoint.x);
    }

    public Point getIntersection(Line other) {
        if (getSlope() == other.getSlope()) {
            return null;
        }
        if (this.isVertical() && !other.isVertical()) {
            return new Point(startPoint.x, startPoint.x * other.getSlope() + other.getYInt());
        } else if (!this.isVertical() && other.isVertical()) {
            return new Point(other.startPoint.x, other.startPoint.x * getSlope() + getYInt());
        }
        //y = slope(x - startPoint.x) + startPoint.y
        //y = slope*x + (startPoint.y - slope*startPoint.x)
        double a = getSlope();
        double b = startPoint.y - getSlope() * startPoint.x;
        double c = other.getSlope();
        double d = other.getYInt();

        double x = (d - b) / (a - c);
        double y = getSlope() * x + getYInt();

        return new Point(x, y);
    }

    public Point closerToEnd(ArrayList<Point> points) {
        double minDistance = Double.MAX_VALUE;
        Point closest = points.get(0);
        for (Point p : points) {
            if (p.distance(endPoint) < minDistance) {
                minDistance = p.distance(endPoint);
                closest = p;
            }
        }
        return closest;
    }

    public ArrayList<Point> pointsOnLine(ArrayList<Point> points) {
        ArrayList<Point> onLine = new ArrayList<>();

        double minX = Math.min(startPoint.x, endPoint.x);
        double maxX = Math.max(startPoint.x, endPoint.x);
        double minY = Math.min(startPoint.y, endPoint.y);
        double maxY = Math.max(startPoint.y, endPoint.y);

        for (Point point : points) {
            boolean withinX = point.x >= minX && point.x <= maxX;
            boolean withinY = point.y >= minY && point.y <= maxY;

            if (withinX && withinY) {
                onLine.add(point);
            }
        }

        return onLine;
    }

    public boolean containsPoint(Point point) {
        double minX = Math.min(startPoint.x, endPoint.x);
        double maxX = Math.max(startPoint.x, endPoint.x);
        double minY = Math.min(startPoint.y, endPoint.y);
        double maxY = Math.max(startPoint.y, endPoint.y);

        boolean withinX = point.x >= minX && point.x <= maxX;
        boolean withinY = point.y >= minY && point.y <= maxY;

        return withinX && withinY;
    }

    public double getAngle() {
        return Math.atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x);
    }

    public double getHeading() {
        return Math.atan2(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
    }

    public double getSlope() {
        if (isVertical()) {
            return Double.MAX_VALUE; // TODO: not use weird nudging like this, make actual cases?
        }
        return (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x);
    }

    public double getYInt() {
        return startPoint.y - getSlope() * startPoint.x;
    }
}