package com.kuriosityrobotics.firstforward.robot.math;

import java.util.ArrayList;

public class Circle {
    public Point center;
    public double radius;

    public Circle(Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Circle() {
    }

    public double centerDistance(Line line) {
        double distance;
        if (line.isVertical()) {
            distance = Math.abs(center.x - line.startPoint.x);
        }else if (line.slope == 0) {
            distance = Math.abs(center.y - line.startPoint.y);
        }else {
            Line perpendicular = new Line(center, -1 / line.slope);
            Point intersection = line.getIntersection(perpendicular);
            distance = center.distance(intersection);
        }
        return distance;
    }

    public boolean intersects(Line line) {
        return centerDistance(line) <= radius;
    }

    public ArrayList<Point> getIntersections(Line line) {
        double x1;
        double y1;
        double x2;
        double y2;
        ArrayList<Point> intersections = new ArrayList<Point>();

        // circle equation is (x-center.x)^2 + (y-center.y)^2 = radius^2
        if (!this.intersects(line)) {
            return intersections;
        } else if (line.isVertical()) { // line equation is x=startPoint.x
            x1 = line.startPoint.x;
            x2 = x1;
            double sqrtExpression = Math.sqrt(radius*radius - Math.pow(line.startPoint.x - center.x, 2));
            y1 = center.y + sqrtExpression;
            y2 = center.y - sqrtExpression;
        } else if (line.slope == 0) { // line equation is y = startPoint.y
            y1 = line.startPoint.y;
            y2 = y1;
            double sqrtExpression = Math.sqrt(radius*radius - Math.pow(line.startPoint.y - center.y, 2));
            x1 = center.x + sqrtExpression;
            x2 = center.x - sqrtExpression;
        } else { // line equation is y = slope*x + yInt
            double a = line.slope*line.slope + 1;
            double b = 2 * line.yInt * line.slope - 2 * center.y * line.slope - 2 * center.x;
            double c = Math.pow(line.yInt - center.y, 2) + center.x*center.x - radius*radius;

            double[] xyValues = quadraticFormula(a, b, c);

            x1 = xyValues[0];
            y1 = x1 * line.slope + line.yInt;
            x2 = xyValues[1];
            y2 = x2 * line.slope + line.yInt;
        }

        intersections.add(new Point(x1, y1));
        if (!(x1 == x2 && y1 == y2)) {
            intersections.add(new Point(x2, y2));
        }
        return intersections;
    }

    // the range is x value
    public ArrayList<Point> getIntersections(Line line, double xMin, double xMax) {
        double yMin = xMin * line.slope + line.yInt;
        double yMax = xMax * line.slope + line.yInt;

        ArrayList<Point> intersectionsInRange = new ArrayList<Point>();
        ArrayList<Point> intersections = this.getIntersections(line);
        if (line.isVertical() && xMin <= line.startPoint.x && line.startPoint.x <= xMax){
            return intersections;
        }

        for (Point p : intersections) {
            if (p.x >= xMin && p.x <= xMax && p.y >= yMin && p.y <= yMax) {
                intersectionsInRange.add(p);
            }
        }
        return intersectionsInRange;
    }

    public static double[] quadraticFormula(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return new double[0];
        }
        return new double[] { (-b + Math.sqrt(discriminant)) / (2 * a), (-b - Math.sqrt(discriminant)) / (2 * a) };
    }
}