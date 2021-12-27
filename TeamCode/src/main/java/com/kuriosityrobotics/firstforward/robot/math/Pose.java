package com.kuriosityrobotics.firstforward.robot.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

public class Pose extends Point {
    public double heading;

    public static Pose DEFAULT_POSE = new Pose(0,0,0);

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

    public DashboardVector getHeadingVector() {
        return new DashboardVector(cos(heading), sin(heading));
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
    public String toString() {
        return "(" + x + ", " + y + ", " + heading + ")";
    }

    public static Pose fuse(Pose p1, Pose p2) {
        return new Pose((p1.x + p2.x) / 2, (p1.x + p2.y) / 2, (p1.heading + p2.heading) / 2);
    }
}
