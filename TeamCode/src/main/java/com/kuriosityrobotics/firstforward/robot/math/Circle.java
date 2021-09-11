package com.kuriosityrobotics.firstforward.robot.math;

import java.util.ArrayList;

public class Circle {
    public Point center;
    public double radius;

    public Circle(Point center, double radius){
        this.center = center;
        this.radius = radius;
    }
    public Circle(){
    }

    public double distance(Line line){
        Line perpendicular = new Line(center, -1/line.slope);
        Point intersection = line.getIntersection(perpendicular);
        return center.distance(intersection);
    }

    public Boolean intersects(Line line){
        return distance(line) <= radius;
    }

    public ArrayList<Point> getIntersections(Line line){
        if (!this.intersects(line)){ return null; }
        //circle equation is (x-center.x)^2 + (y-center.y)^2 = radius^2
        //line equation is y = slope*x + yInt

        double a = Math.pow(line.slope, 2) + 1;
        double b = 2*line.yInt*line.slope - 2*center.y*line.slope - 2*center.x;
        double c = center.x*center.x - 2*line.yInt*center.y + center.y*center.y - radius*radius;

        double[] xyValues = quadraticFormula(a, b, c);

        double x1 = xyValues[0];
        double y1 = x1*line.slope + line.yInt;
        double x2 = xyValues[1];
        double y2 = x2*line.slope + line.yInt;

        ArrayList<Point> intersections = new ArrayList<Point>();
        intersections.add(new Point(x1, y1));
        intersections.add(new Point(x2, y2));
        return intersections;
    }
    //the range is x value
    public ArrayList<Point> getIntersections(Line line, double xMin, double xMax){
        if (!this.intersects(line)){ return null; }
        //circle equation is (x-center.x)^2 + (y-center.y)^2 = radius^2
        //line equation is y = slope*x + yInt
        double yMin = xMin*line.slope + line.yInt;
        double yMax = xMax*line.slope + line.yInt;

        ArrayList<Point> intersections = this.getIntersections(line);
        Point p1 = intersections.get(0);
        Point p2 = intersections.get(1);

        ArrayList<Point> intersectionsInRange = new ArrayList<Point>();
        if (p1.x >= xMin && p1.x <= xMax && p1.y >= yMin && p1.y <= yMax){
            intersectionsInRange.add(p1);
        }
        if (p2.x >= xMin && p2.x <= xMax && p2.y >= yMin && p2.y <= yMax){
            intersectionsInRange.add(p2);
        }
        return intersectionsInRange;
    }
    public static double[] quadraticFormula(double a, double b, double c){
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0){ return null; }
        return new double[]{
                (-b + Math.sqrt(discriminant))/(2*a),
                (-b - Math.sqrt(discriminant))/(2*a)
        };
    }
}
