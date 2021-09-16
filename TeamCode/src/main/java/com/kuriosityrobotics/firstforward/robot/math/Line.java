package com.kuriosityrobotics.firstforward.robot.math;

import java.util.ArrayList;

public class Line {
    public Point startPoint;
    public Point endPoint = null;
    public double slope;
    public double yInt;

    //for pathfollow
    public Line(Point startPoint, Point endPoint){
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        if (Math.abs(endPoint.x - startPoint.x) < .0001) {
            //set to 0 since it needs to be init-ed
            slope = 0;
        }
        slope = (endPoint.y - startPoint.y)/(endPoint.x - startPoint.x);
        yInt = startPoint.y - slope*startPoint.x;
    }
    //for finding intersections
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
        Point closest = new Point();
        for (Point p : points) {
            if (p.distance(endPoint) < minDistance) {
                minDistance = p.distance(endPoint);
                closest = p;
            }
        }
        return closest;
    }
}