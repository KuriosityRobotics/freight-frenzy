package com.kuriosityrobotics.firstforward.robot.util.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;
import org.ojalgo.matrix.Primitive64Matrix;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Map;

public class MathUtil {
    private static final double EPSILON = 0.001;
    private static final Precision.DoubleEquivalence PRECISION = Precision.doubleEquivalenceOfEpsilon(EPSILON);

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

    public static boolean inRange(double min, double max, double value) {
        return PRECISION.gte(value, min)
                && PRECISION.lte(value, max);
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

    public static double median(Collection<Map.Entry<Long, Double>> t) {
        var newCollection = new ArrayList<>(t);
        Collections.sort(newCollection, Comparator.comparingDouble(Map.Entry::getValue));
        return newCollection.get(newCollection.size() / 2).getValue();
    }


    public static double sd(Collection<Double> t) {
        return Math.sqrt(
                t.stream()
                        .mapToDouble(n -> Math.pow(n - mean(t), 2))
                        .sum() / t.size()
        );
    }

    public static Primitive64Matrix rotate(double theta) {
        return Primitive64Matrix.FACTORY.rows(new double[][]{
                {cos(theta), sin(theta), 0},
                {-sin(theta), cos(theta), 0},
                {0, 0, 1}
        });
    }

    public static Vector2D rotate(Vector2D vector, double angle) {
        return Vector2D.of(
                cos(angle) * vector.getX() + sin(angle) * vector.getY(),
                -sin(angle) * vector.getX() + cos(angle) * vector.getY()
        );
    }

    public static double truncate(double x, double numDecimals) {
        double pow = Math.pow(10 ,numDecimals);
        return Math.floor(x * pow) / pow;
    }
}
