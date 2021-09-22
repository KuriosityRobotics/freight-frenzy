package com.kuriosityrobotics.firstforward.robot.math;

import java.util.ArrayList;

public class Line {
    public Point startPoint;
    public Point endPoint = null;
    public double slope;
    public double yInt;

    //for pathfollow (LINE SEGMENT)
    public Line(Point startPoint, Point endPoint){
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        if (Math.abs(endPoint.x - startPoint.x) < .0001) {
            //it needs to be init-ed
            slope = 9999999;
        }
        slope = (endPoint.y - startPoint.y)/(endPoint.x - startPoint.x);
        yInt = startPoint.y - slope*startPoint.x;
    }
    //for finding intersections (LINE)
    public Line(Point point, double slope){
        this.startPoint = point;
        this.slope = slope;
        yInt = point.y - slope*point.x;
    }
    //null line
    public Line(){
    }

    public boolean isVertical() {
        boolean vertical;
        if (endPoint == null) {
            vertical = false;
        }else {
            vertical = Math.abs(endPoint.x - startPoint.x) < .0001;
        }
        return vertical;
    }

    public Point getIntersection(Line other){
        if (slope == other.slope){ return null; }
        if (this.isVertical() && !other.isVertical()) {
            return new Point(startPoint.x, startPoint.x*other.slope + other.yInt);
        }else if (!this.isVertical() && other.isVertical()) {
            return new Point(other.startPoint.x, other.startPoint.x*slope + yInt);
        }
        //y = slope(x - startPoint.x) + startPoint.y
        //y = slope*x + (startPoint.y - slope*startPoint.x)
        double a = slope;
        double b = startPoint.y - slope*startPoint.x;
        double c = other.slope;
        double d = other.yInt;

        double x = (d-b)/(a-c);
        double y = slope*x + yInt;

        return new Point(x, y);
    }

    public Point closerToEnd(ArrayList<Point> points){
        double minDistance = 999999999;
        Point closest = points.get(0);
        for (Point p : points) {
            if (p.distance(endPoint) < minDistance) {
                minDistance = p.distance(endPoint);
                closest = p;
            }
        }
        return closest;
    }

    public ArrayList<Point> pointsInRange(ArrayList<Point> points, double xMin, double xMax){
        ArrayList<Point> inRange = new ArrayList<Point>();

        if (isVertical() && xMin <= startPoint.x && startPoint.x <= xMax){
            return points;
        }
        for (Point p : points) {
            if (p.isOnSegment(new Line(new Point(xMin, slope*xMin + yInt), new Point(xMax, slope*xMax + yInt)))) {
                inRange.add(p);
            }
        }
        return inRange;
    }

    public double segmentDistance(Point p, double xMin, double xMax) {
        return 0;
    }
}