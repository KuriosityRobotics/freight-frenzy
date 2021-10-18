package com.kuriosityrobotics.firstforward.robot.math;

public class MathFunctions {
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
        int times = (int) (angle / (2 * Math.PI));
        angle -= times * 2 * Math.PI;

        // angle is between 0 and 360, change so its between -180 and 180
        if (angle < -Math.PI) {
            angle += (2 * Math.PI);
        }
        else if (angle > Math.PI) {
            angle -= (2 * Math.PI);
        }

        double result = angle + centerOfWrap;
        return result;
    }
}