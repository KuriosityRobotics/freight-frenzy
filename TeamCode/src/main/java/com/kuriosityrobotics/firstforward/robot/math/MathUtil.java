package com.kuriosityrobotics.firstforward.robot.math;

public class MathUtil {
    private static final double EPSILON = 0.00001;

    /**
     * Wraps angle (in radians) to a value from -pi to pi
     *
     * @param angle Angle to be wrapped
     * @return The wrapped angle, between -pi and pi
     */
    public static double angleWrap(double angle) {
        return angleWrap(angle, 0);
    }

    /**
     * Wraps an angle (in radians) to a value within pi of centerOfWrap.
     *
     * @param angle        The angle to be wrapped
     * @param centerOfWrap The center of the boundary in which the angle can be wrapped to.
     * @return The wrapped angle, which will lie within pi of the centerOfWrap.
     */
    public static double angleWrap(double angle, double centerOfWrap) {
        if (angle > centerOfWrap + Math.PI) {
            return angleWrap(angle - (2 * Math.PI), centerOfWrap);
        } else if (angle < centerOfWrap - Math.PI) {
            return angleWrap(angle + (2 * Math.PI), centerOfWrap);
        } else {
            return angle;
        }
    }

    public static boolean doublesEqual(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static double max(double... in) {
        double max = Double.MIN_VALUE;
        for (double x : in) {
            max = Math.max(max, x);
        }

        return max;
    }
}
