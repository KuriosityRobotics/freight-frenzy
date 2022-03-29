package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.Robot.assertThat;
import static com.kuriosityrobotics.firstforward.robot.sensors.DistanceSensors.PhysicalDistanceSensor.BACK;
import static com.kuriosityrobotics.firstforward.robot.sensors.DistanceSensors.PhysicalDistanceSensor.LEFT;
import static com.kuriosityrobotics.firstforward.robot.sensors.DistanceSensors.PhysicalDistanceSensor.RIGHT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.ROBOT_HEIGHT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.ROBOT_WIDTH;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AsynchProcess;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Ray;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ojalgo.matrix.Primitive64Matrix;

import kotlin.NotImplementedError;

public class DistanceSensors {
    static final Precision.DoubleEquivalence p = Precision.doubleEquivalenceOfEpsilon(.05);
    public static final Segment leftWall = Lines.fromPointAndDirection(Vector2D.of(0, 0), Vector2D.of(0, 1), p).segment(0, FULL_FIELD);
    public static final Segment rightWall = Lines.fromPointAndDirection(Vector2D.of(140.5, 0), Vector2D.of(0, 1), p).segment(0, FULL_FIELD);
    public static final Segment bottomWall = Lines.fromPointAndDirection(Vector2D.of(0, 0), Vector2D.of(1, 0), p).segment(0, FULL_FIELD);
    public static final Segment topWall = Lines.fromPointAndDirection(Vector2D.of(0, 140.5), Vector2D.of(1, 0), p).segment(0, FULL_FIELD);

    private final AsynchProcess[] sensors;
    private final LocationProvider locationProvider;
    private final ExtendedKalmanFilter filter;

    public DistanceSensors(HardwareMap hardwareMap, LocationProvider locationProvider, ExtendedKalmanFilter filter) {
        this.sensors = new AsynchProcess[]{
                asynchDistanceSensor(LEFT, hardwareMap.get(DistanceSensor.class, "dleft")),
                asynchDistanceSensor(RIGHT, hardwareMap.get(DistanceSensor.class, "dright")),
                asynchDistanceSensor(BACK, hardwareMap.get(DistanceSensor.class, "dback"))
        };
        this.locationProvider = locationProvider;
        this.filter = filter;
    }

    // TODO:  dont use asynchprocess in thsi class
    private AsynchProcess asynchDistanceSensor(PhysicalDistanceSensor physical, DistanceSensor sensor) {
        return AsynchProcess.parallel(() -> {
            var dist = sensor.getDistance(DistanceUnit.INCH);
            var pose = physical.getRobotPose(dist, locationProvider.getPose().heading);
            filter.datumBuilder()
                    .mean(pose.x, pose.y)
                    .outputToState(Primitive64Matrix.FACTORY.rows(new double[][]{
                            {1, 0, 0},
                            {0, 1, 0}
                    }))
                    .variance(dist/20., dist/20.)
                    .correct();
        }, 5);
    }

    void update() {
        for (AsynchProcess sensor : sensors)
            sensor.update();

    }

    enum PhysicalDistanceSensor {
        LEFT(new Point(-ROBOT_WIDTH / 2, 0)),
        RIGHT(new Point(ROBOT_WIDTH / 2, 0)),
        BACK(new Point(0, -ROBOT_HEIGHT / 2));

        private final Point relativePose;

        PhysicalDistanceSensor(Point relativePose) {
            this.relativePose = relativePose;
        }

        private static Point c(Vector2D p) {
            return new Point(p.getX(), p.getY());
        }

        private static Vector2D c(Point p) {
            return Vector2D.of(p.x, p.y);
        }

        public Segment bestWall(Pose robotPosition) {
            var ray = getDistanceSensorRay(robotPosition);

            var iLeft = ray.intersection(leftWall);
            var iRight = ray.intersection(rightWall);
            var iBottom = ray.intersection(bottomWall);
            var iTop = ray.intersection(topWall);

            if (iLeft != null)
                return leftWall;
            else if (iRight != null)
                return rightWall;
            else if (iBottom != null)
                return bottomWall;
            else if (iTop != null)
                return topWall;
            else {
                assertThat(false, "No intersections with boundaries");
                return null; // attempt recovery
            }
        }

        private Ray getDistanceSensorRay(Pose robotPosition) {
            var rotatedRelative = relativePose.rotate(robotPosition.heading);
            return Lines.rayFromPointAndDirection(c(robotPosition), c(rotatedRelative), p);
        }

        public Pose getRobotPose(double distInches, double heading) {
            throw new NotImplementedError(); // TODO:  get position from dist sensor readings
        }
    }


}
