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
    /**
     * Creates a point with null x and y
     */
    public Point() {
    }

    public double distance(Point other){
        return Math.sqrt((Math.pow((other.x - x), 2) + Math.pow((other.y - y), 2)));
    }

    @Override
    public String toString() {
        return x + ", " + y;
    }

    @Override
    public boolean equals(Object point) {
        return ((Point) point).x == x && ((Point) point).y == y;
    }
}
