package com.kuriosityrobotics.firstforward.robot.math;

public class MathFunctions {
    public static double round(double a){
        return round(a,2);
    }

    public static double round(double a, int digits){
        return Math.round(a * Math.pow(10.0,digits)) / Math.pow(10.0,digits);
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
        if (angle >= centerOfWrap + Math.PI) {
            return angleWrap(angle - (2 * Math.PI), centerOfWrap);
        } else if (angle < centerOfWrap - Math.PI) {
            return angleWrap(angle + (2 * Math.PI), centerOfWrap);
        } else {
            return angle;
        }
    }

    /*
    taylor series expansion to make stuff COOL
     */
    public static double sinXOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = 1;
            double bottom = 1;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 2) * (2 * i + 3);
            }
            return retVal;
        } else {
            return Math.sin(x) / x;
        }
    }

    /*
    taylor series expansion to make stuff COOL
     */
    public static double cosXMinusOneOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = -x;
            double bottom = 2;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 3) * (2 * i + 4);
            }
            return retVal;
        } else {
            return (Math.cos(x) - 1) / x;
        }
    }
}