package com.kuriosityrobotics.firstforward.robot.sensors.kf;
import org.ojalgo.matrix.Primitive64Matrix;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import java.time.Instant;

public class KalmanData {
    private KalmanData() {
    }

    private static Primitive64Matrix rotate(double theta) {
        return Primitive64Matrix.FACTORY.rows(new double[][]{
                {cos(theta), sin(theta), 0},
                {-sin(theta), cos(theta), 0},
                {0, 0, 1}
        });
    }

    public static KalmanDatum odometryDatum(Instant time, double globalHeading, double dx, double dy, double dtheta) {
        return new KalmanDatum(time, rotate(globalHeading).invert(),
                new double[]{dx, dy, dtheta}, new double[]{pow(.8 * dx, 2), pow(.8 * dy, 2), pow(.8 * dtheta, 2)});
    }

    public static KalmanDatum gyroDatum(Instant time, double globalHeading) {
        return new KalmanDatum(
                time,
                Primitive64Matrix.FACTORY.column(globalHeading),
                Primitive64Matrix.FACTORY.column(toRadians(3)),
                Primitive64Matrix.FACTORY.row(0, 0, 1));
    }

    public static KalmanDatum vuforiaDatum(Pose pose) {
        return new KalmanDatum(new double[]{pose.x, pose.y, pose.heading}, new double[]{.04, .04, 0.00121847});
    }
}
