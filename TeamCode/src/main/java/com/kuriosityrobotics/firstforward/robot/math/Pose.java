package com.kuriosityrobotics.firstforward.robot.math;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Locale;

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

    public boolean isInRange(double xMin, double yMin, double xMax, double yMax) {
        return (this.x >= xMin) && (this.x <= xMax) && (this.y >= yMin) && (this.y <= yMax);
    }

    public Pose toDegrees() {
        return new Pose(this.x, this.y, Math.toDegrees(this.heading));
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "x = %.3f, y = %.3f, heading = %.3f", x, y, heading);
    }

    // sus naming but whatever
    public Pose toFTCSystem() {
        double x =  -this.y + HALF_FIELD / MM_PER_INCH;
        double y = this.x - HALF_FIELD / MM_PER_INCH;
        double heading = angleWrap(this.heading - Math.PI);
        return new Pose(x, y, heading);
    }
}
