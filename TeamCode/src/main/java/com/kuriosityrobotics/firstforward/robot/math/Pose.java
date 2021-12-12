package com.kuriosityrobotics.firstforward.robot.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        super(point.x, point.y);
        this.heading = heading;
    }

    public Pose(Pose pose) {
        this(pose.x, pose.y, pose.heading);
    }

    public Vector2d getHeadingVector() {
        return new Vector2d(cos(heading), sin(heading));
    }

    public Pose difference(Pose lastPose) {
        return new Pose(this.x - lastPose.x, this.y - lastPose.y, this.heading - lastPose.heading);
    }

    public Pose add(Pose other) {
        return new Pose(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    public Pose divide(double a) {
        return new Pose(this.x / a, this.y / a, this.heading / a);
    }

    @NonNull
    @Override
    // escape sequence formatting moment
    public String toString() {
        return "\nx: " + x +
                ", \ny: " + y +
                ", \nheading:" + heading;
    }
}
