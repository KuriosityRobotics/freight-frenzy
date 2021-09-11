package com.kuriosityrobotics.firstforward.robot.math;

public class Line {
    public Point startPoint;
    public Point endPoint;
    public double slope;
    public double yInt;

    //for pathfollow
    public Line(Point startPoint, Point endPoint){
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        slope = (endPoint.y - startPoint.y)/(endPoint.x - startPoint.x);
        yInt = startPoint.y - slope*startPoint.x;
    }
    //for finding intersections
    public Line(Point point, double slope){
        this.startPoint = point;
        this.slope = slope;
    }
    //null line
    public Line(){
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

    public Point closerToEnd(Point p1, Point p2){
        if (p1.distance(endPoint) < p2.distance(endPoint)){
            return p1;
        }
        return p2;
    }
}
