package com.kuriosityrobotics.firstforward.robot.util.math;

import java.util.Collection;

public class MathUtil {
    private static final double EPSILON = 0.00001;

    public static int toNum(boolean value) {
        return value ? 1 : 0;
    }

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
        angle -= centerOfWrap;

        double newAng = angle - ((2 * Math.PI) * Math.round(angle / (2 * Math.PI)));

        return newAng + centerOfWrap;

//        angle -= centerOfWrap;
//        int times = (int) (angle / (2 * Math.PI));
//        angle -= times * 2 * Math.PI;
//
//        // angle is between 0 and 360, change so its between -180 and 180
//        if (angle < -Math.PI) {
//            angle += (2 * Math.PI);
//        }
//        else if (angle > Math.PI) {
//            angle -= (2 * Math.PI);
//        }
//
//        return angle + centerOfWrap;
    }

    public static boolean doublesEqual(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static boolean doublesEqualEpsilon(double epsilon, double a, double b) {
        return Math.abs(a - b) < epsilon;
    }

    public static double max(double... in) {
        double max = Double.MIN_VALUE;
        for (double x : in) {
            max = Math.max(max, x);
        }

        return max;
    }

    public static double mean(Collection<Double> t, int start, int maxCount) {
        return t.stream()
                .skip(start)
                .limit(maxCount)
                .mapToDouble(n -> n)
                .average().orElse(0);
    }

    public static double mean(Collection<Double> t) {
        return mean(t, 0, t.size());
    }

    public static double sd(Collection<Double> t) {
        return Math.sqrt(
                t.stream()
                        .mapToDouble(n -> Math.pow(n - mean(t), 2))
                        .sum() / t.size()
        );
    }
}
