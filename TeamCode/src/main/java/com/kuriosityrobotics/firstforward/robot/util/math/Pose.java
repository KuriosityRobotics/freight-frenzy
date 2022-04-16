package com.kuriosityrobotics.firstforward.robot.util.math;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.truncate;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;

// Kuro coordinate system pose :tm:
public class Pose extends Point {
    public static final Pose ZERO = new Pose(0, 0, 0);
    public final double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public static Pose of(double[] values) {
        if (values.length < 3)
            return Pose.ZERO;

        return new Pose(values[0], values[1], values[2]);
    }

    public Pose fieldMirror() {
        return fieldMirror(this.x, this.y, this.heading);
    }

    public static Pose fieldMirror(double x, double y, double heading) {
        return new Pose(FULL_FIELD - x, y, angleWrap(-heading));
    }

    public static Pose relativeMirror(double x, double y, double heading) {
        return new Pose(-x, y, angleWrap(-heading));
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

    public Pose minus(Pose lastPose) {
        return new Pose(this.x - lastPose.x, this.y - lastPose.y, this.heading - lastPose.heading);
    }

    public Pose add(Pose other) {
        return new Pose(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    public Pose divide(double a) {
        return new Pose(this.x / a, this.y / a, this.heading / a);
    }

    public Pose between(Pose other) {
        return new Pose(this.x / 2 + other.x / 2, this.y / 2 + other.y / 2, (this.heading + other.heading) / 2);
    }

    public Pose wrapped() {
        return new Pose(x, y, angleWrap(heading));
    }

    public Pose scale(double scale) {
        return new Pose(super.scale(scale), heading * scale);
    }

    public boolean isInRange(double xMin, double yMin, double xMax, double yMax) {
        return (this.x >= xMin) && (this.x <= xMax) && (this.y >= yMin) && (this.y <= yMax);
    }

    @NonNull
    @Override
    public String toString() {
        return toString("");
    }

    public String toString(String label) {
//        return String.format(Locale.US, "%-10s x: %,6.2f y: %,6.2f θ: %,3.0f", label, x, y, toDegrees(angleWrap(heading)))
//                .replace(" ", "\u00a0");
        return (label + " x: " + truncate(x, 2) + " y: " + truncate(y, 2) + " θ: " + toDegrees(angleWrap(heading)))
                .replace(" ", "\u00a0");
    }

    // sus naming but whatever
    public Pose toFTCSystem() {
        double x = -this.y + (FULL_FIELD / 2.);
        double y = this.x - (FULL_FIELD / 2.);
        double heading = angleWrap(Math.PI - this.heading);
        return new Pose(x, y, heading);
    }

    /**
     * Get the global absolute angle of the line between the robot's position and the given point,
     * where 0 is along the y axis.
     *
     * @return absolute heading to that point
     */
    public double absoluteHeadingToPoint(Point point) {
        return Math.atan2(point.x - this.x, point.y - this.y);
    }

    /**
     * The heading a robot at this pose would have to turn by to face the point directly. Returned value is angleWrapped.
     *
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

    public Point toPoint() {
        return new Point(this.x, this.y);
    }
}
