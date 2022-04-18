package com.kuriosityrobotics.firstforward.robot.sensors;

import static java.lang.Math.pow;
import static java.lang.Math.toRadians;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.SharpIRDistance;

import org.apache.commons.numbers.core.Precision;
import org.ojalgo.matrix.Primitive64Matrix;

import java.util.ArrayList;
import java.util.List;

public class DistanceSensorLocaliser implements Module, LocationProvider, Telemeter {
    public static final double SENSOR_X_DISPLACEMENT_RIGHT = 5.354;
    public static final double SENSOR_X_DISPLACEMENT_LEFT = 4.75;
    public static final double BACK_SENSORS_Y_DISPLACEMENT = 5.6345;
    public static final double FRONT_SENSORS_Y_DISPLACEMENT = 2.9505;
    private static final double DISTANCE_SENSOR_MARGIN_OF_ERROR = .01;

    static final Precision.DoubleEquivalence p = Precision.doubleEquivalenceOfEpsilon(.05);

    private final LocationProvider locationProvider;
    private final ExtendedKalmanFilter filter;
    private final SharpIRDistance frontLeft, backLeft, frontRight, backRight;

    private double x, y, heading;

    public DistanceSensorLocaliser(LocationProvider locationProvider, ExtendedKalmanFilter filter,
                                   SharpIRDistance frontLeft, SharpIRDistance backLeft,
                                   SharpIRDistance frontRight, SharpIRDistance backRight) {
        this.locationProvider = locationProvider;
        this.filter = filter;

        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
    }

    private void processPartialState(Double[] sensorData) {
        if (sensorData == null)
            return;
        heading = sensorData[2];

        Primitive64Matrix H;
        double dist;
        if (sensorData[0] != null) {
            dist = x = sensorData[0];
            H = Primitive64Matrix.FACTORY.rows(new double[][]{
                    {1, 0, 0}, // x
                    {0, 0, 1} // heading
            });
        } else if (sensorData[1] != null) {
            dist = y = sensorData[1];
            H = Primitive64Matrix.FACTORY.rows(new double[][]{
                    {0, 1, 0}, // y
                    {0, 0, 1} // heading
            });
        } else
            return;

        var sensorVariance = 2 * pow(dist * DISTANCE_SENSOR_MARGIN_OF_ERROR, 2); // this is ok because dist is a linear combination of the two sensors
        var angleVariance = toRadians(6.674 * 6.674) * sensorVariance; // 6.674 is derivative of getWallHeading(k) at point x=0.  then apply error propagation law to approximated linear function
        filter.builder()
                .mean(dist, heading)
                .stateToOutput(H)
                .variance(
                        sensorVariance,
                        angleVariance
                )
                .correct();
    }

    public void update() {
        {
            var bestWallLeft = DistanceSensorPair.LEFT.bestWall(Pose.of(filter.getVariance()), locationProvider.getPose());
            if (bestWallLeft != null)
                processPartialState(DistanceSensorPair.LEFT.getPartialState(
                        frontLeft.getDistance(),
                        backLeft.getDistance(),
                        bestWallLeft
                ));
        }

        {
            var bestWallRight = DistanceSensorPair.RIGHT.bestWall(Pose.of(filter.getVariance()), locationProvider.getPose());
            if (bestWallRight != null)
                processPartialState(DistanceSensorPair.RIGHT.getPartialState(
                        frontRight.getDistance(),
                        backRight.getDistance(),
                        bestWallRight
                ));
        }
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "DistanceSensorLocaliser";
    }

    @Override
    public Pose getPose() {
        return new Pose(x, y, heading);
    }

    @Override
    public Pose getVelocity() {
        throw new UnsupportedOperationException("DistanceSensorLocaliser cannot provide robot velocity.");
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add(getPose().toString("Robot pose"));
        }};
    }

}
