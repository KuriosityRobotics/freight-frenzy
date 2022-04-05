package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static java.lang.Math.pow;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ojalgo.matrix.Primitive64Matrix;

import java.util.ArrayList;
import java.util.List;

public class DistanceSensors implements Module, LocationProvider {
    public static final double SENSOR_X_DISPLACEMENT = 5.354;
    public static final double BACK_SENSORS_Y_DISPLACEMENT = 5.6345;
    public static final double FRONT_SENSORS_Y_DISPLACEMENT = 2.9505;
    private static final double DISTANCE_SENSOR_MARGIN_OF_ERROR = .01;

    static final Precision.DoubleEquivalence p = Precision.doubleEquivalenceOfEpsilon(.05);

    private final LocationProvider locationProvider;
    private final ExtendedKalmanFilter filter;
    private final DistanceSensor frontLeft, backLeft, frontRight, backRight;

    private double x, y, heading;

    public DistanceSensors(HardwareMap hardwareMap, LocationProvider locationProvider, ExtendedKalmanFilter filter) {
        this.locationProvider = locationProvider;
        this.filter = filter;

        this.frontLeft = hardwareMap.get(DistanceSensor.class, "frontLeft");
        this.backLeft = hardwareMap.get(DistanceSensor.class, "backLeft");
        this.frontRight = hardwareMap.get(DistanceSensor.class, "frontRight");
        this.backRight = hardwareMap.get(DistanceSensor.class, "backRight");
    }

    private void processPartialState(Double[] sensorData) {
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
            var bestWallLeft = DistanceSensorPair.LEFT.bestWall(locationProvider.getPose());
            if (bestWallLeft != null)
                processPartialState(DistanceSensorPair.LEFT.getPartialState(
                        frontLeft.getDistance(DistanceUnit.INCH),
                        backLeft.getDistance(DistanceUnit.INCH),
                        bestWallLeft
                ));
        }

        {
            var bestWallRight = DistanceSensorPair.RIGHT.bestWall(locationProvider.getPose());
            if (bestWallRight != null)
                processPartialState(DistanceSensorPair.RIGHT.getPartialState(
                        frontRight.getDistance(DistanceUnit.INCH),
                        backRight.getDistance(DistanceUnit.INCH),
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
        return "DistanceSensors";
    }

    @Override
    public Pose getPose() {
        return new Pose(x, y, heading);
    }

    @Override
    public Pose getVelocity() {
        throw new UnsupportedOperationException("DistanceSensors cannot provide robot velocity.");
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("Robot pose:  " + getPose());
        }};
    }

    @Override
    public int maxFrequency() {
        return 10;
    }

    public enum Wall {
        RIGHT(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(0, FULL_FIELD), p)),
        LEFT(Lines.segmentFromPoints(Vector2D.of(FULL_FIELD, 0), Vector2D.of(FULL_FIELD, FULL_FIELD), p)),
        FRONT(Lines.segmentFromPoints(Vector2D.of(0, FULL_FIELD), Vector2D.of(FULL_FIELD, FULL_FIELD), p)),
        BACK(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(FULL_FIELD, 0), p));

        private final Segment segment;

        Wall(Segment segment) {
            this.segment = segment;
        }

        public Segment getSegment() {
            return segment;
        }
    }
}
