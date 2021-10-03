package com.kuriosityrobotics.firstforward.robot.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Vector2d getHeadingVector() {
        return new Vector2d(cos(heading), sin(heading));
    }

    @NonNull
    @Override
    public String toString() {
        return "x=" + x +
                ", y=" + y +
                ", heading=" + heading;
    }
}
