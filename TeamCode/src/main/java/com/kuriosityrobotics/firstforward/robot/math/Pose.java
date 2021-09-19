package com.kuriosityrobotics.firstforward.robot.math;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    @Override
    public String toString() {
        return "x=" + x +
                ", y=" + y +
                ", heading=" + heading;
    }
}
