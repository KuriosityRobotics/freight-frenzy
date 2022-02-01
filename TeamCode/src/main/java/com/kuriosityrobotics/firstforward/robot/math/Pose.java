package com.kuriosityrobotics.firstforward.robot.math;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.MM_PER_INCH;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.kuriosityrobotics.firstforward.robot.util.Constants;

import java.util.Locale;

// Kuro coordinate system pose :tm:
public class Pose extends Point {
    public static final Pose ZERO = new Pose(0, 0, 0);
    public final double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        this(point.x, point.y, heading);
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

    public Pose between(Pose other){ return new Pose(this.x/2 + other.x/2, this.y/2 + other.y/2, other.heading); }

    public Pose wrapped() {
        return new Pose(x, y, angleWrap(heading));
    }

    public Pose scale(double scale) {
        return new Pose(super.scale(scale), heading * scale);
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
        double x = -this.y + Constants.FULL_FIELD / 2 / MM_PER_INCH;
        double y = this.x - Constants.FULL_FIELD / 2 / MM_PER_INCH;
        double heading = angleWrap(this.heading - Math.PI);
        return new Pose(x, y, heading);
    }

    /**
     * Get the global absolute angle of the line between the robot's position and the given point,
     * where 0 is along the y axis.
     *
     * @param point
     * @return absolute heading to that point
     */
    public double absoluteHeadingToPoint(Point point) {
        return Math.atan2(point.x - this.x, point.y - this.y);
    }

    /**
     * The heading the robot would have to turn by to face the point directly.
     *
     * @param point
     * @return relative heading to that point
     */
    public double relativeHeadingToPoint(Point point) {
        double absoluteHeadingToPoint = absoluteHeadingToPoint(point);
        return angleWrap(absoluteHeadingToPoint - this.heading);
    }

    public Point relativeComponentsToPoint(Point point) {
        double relativeHeadingToPoint = relativeHeadingToPoint(point);

        double distanceError = this.distance(point);

        double xError = distanceError * Math.sin(relativeHeadingToPoint);
        double yError = distanceError * Math.cos(relativeHeadingToPoint);

        return new Point(xError, yError);
    }

    public double getHeading() {
        return heading;
    }
}
