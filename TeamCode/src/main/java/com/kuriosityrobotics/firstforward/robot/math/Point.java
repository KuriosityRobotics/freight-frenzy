package com.kuriosityrobotics.firstforward.robot.math;

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

    public double distance(Point other){
        return Math.hypot(other.x - x, other.y - y);
    }

    //SEGMENTS

    public boolean isOnSegment(Line line) {
        double xMin = Math.min(line.startPoint.x, line.endPoint.x);
        double xMax = Math.max(line.startPoint.x, line.endPoint.x);
        double yMin;
        double yMax;
        if (line.isVertical()) {
            return y >= xMin && y <= xMax && x - line.startPoint.x < .0001;
        }else {
            yMin = line.slope*xMin + line.yInt;
            yMax = line.slope*xMax + line.yInt;
            return x >= xMin && x <= xMax &&
                    y >= yMin && y <= yMax &&
                    line.startPoint.y - line.slope*line.startPoint.x - line.yInt < .00001;
        }
    }

    //SEGMENTS
    public Point getNearestPoint(Line line) {
        Point nearest;
        if (line.isVertical()) {
            nearest = new Point(line.startPoint.x, y);
        }else if (line.slope == 0) {
            nearest = new Point(x, line.startPoint.y);
        }else {
            Line perpendicular = new Line(this, -1/line.slope);
            nearest = line.getIntersection(perpendicular);
        }

        if (nearest.isOnSegment(line)) {
            return nearest;
        }else {
            if (nearest.distance(line.startPoint) <= nearest.distance(line.endPoint)) {
                return line.startPoint;
            }
            return line.endPoint;
        }
    }

    //SEGMENTS
    public double segmentDistance(Line line) {
        return this.distance(getNearestPoint(line));
    }

    @Override
    public String toString() {
        return x + ", " + y;
    }

    @Override
    public boolean equals(Object point) {
        return Math.abs(((Point) point).x - x) < .00001 &&
                Math.abs(((Point) point).y - y) < .00001;
    }
}

