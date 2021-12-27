package com.kuriosityrobotics.firstforward.robot.math;

// of course this isn't the definition of an actual vector, it's just so that I can draw stuff on the dashboard without using ACME's dependency
// This is a simplified version that only includes the stuff we need
public class DashboardVector {
    private double x;
    private double y;

    public DashboardVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public DashboardVector times(double scalar) {
        return new DashboardVector(scalar * x, scalar * y);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }
}
