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

        private final Point relativePose1, relativePose2;

        PhysicalDistanceSensor(Point relativePose1, Point relativePose2) {
            this.relativePose1 = relativePose1;
            this.relativePose2 = relativePose2;
        }

        private static Point c(Vector2D p) {
            return new Point(p.getX(), p.getY());
        }

        private static Vector2D c(Point p) {
            return Vector2D.of(p.x, p.y);
        }

        public static final double SENSOR_X_DISPLACEMENT = 5.354;
        public static final double BACK_SENSORS_Y_DISPLACEMENT = 5.6345;
        public static final double FRONT_SENSORS_Y_DISPLACEMENT = 2.9505;
        public enum SensorPair {
            RIGHT(
                    Vector2D.of(-SENSOR_X_DISPLACEMENT, -BACK_SENSORS_Y_DISPLACEMENT),
                    Vector2D.of(-SENSOR_X_DISPLACEMENT, FRONT_SENSORS_Y_DISPLACEMENT)),
            LEFT(Vector2D.of(SENSOR_X_DISPLACEMENT, -BACK_SENSORS_Y_DISPLACEMENT),
                    Vector2D.of(SENSOR_X_DISPLACEMENT, FRONT_SENSORS_Y_DISPLACEMENT));


            private final Vector2D offset1, offset2;

            SensorPair(Vector2D offset1, Vector2D offset2) {
                this.offset1 = offset1;
                this.offset2 = offset2;
            }

            public static double getWallHeading(double frontSensor, double backSensor) {
                return atan2(frontSensor - backSensor, FRONT_SENSORS_Y_DISPLACEMENT + BACK_SENSORS_Y_DISPLACEMENT);
            }

            public static double getWallDistance(double frontSensor, double backSensor) {
                double wallHeading = getWallHeading(frontSensor, backSensor);
                double d1 = frontSensor * cos(wallHeading);
                double d2 = backSensor * cos(wallHeading);
                double A = FRONT_SENSORS_Y_DISPLACEMENT / (FRONT_SENSORS_Y_DISPLACEMENT + BACK_SENSORS_Y_DISPLACEMENT);
                double B = BACK_SENSORS_Y_DISPLACEMENT / (FRONT_SENSORS_Y_DISPLACEMENT + BACK_SENSORS_Y_DISPLACEMENT);

                double d = A * d1 + B * d2;
                double dOffset = SENSOR_X_DISPLACEMENT / cos(wallHeading);

                return d + dOffset;
            }

            public static Double[] getFullState(double frontSensor, double backSensor, Wall wall, SensorPair sensorSide) {
                var wallHeading = getWallHeading(frontSensor, backSensor);
                var wallDistance = getWallDistance(frontSensor, backSensor);
                double x = 0.0;
                double y = 0.0;
                double heading = 0;

                switch (wall) {
                    case FRONT -> {
                        y = FULL_FIELD - wallDistance;
                        if (sensorSide == LEFT) {
                            heading = PI / 2 + wallHeading;
                        } else {
                            heading = -PI / 2 - wallHeading;
                        }
                    }
                    case RIGHT -> {
                        x = FULL_FIELD - wallDistance;
                        if (sensorSide == LEFT) {
                            heading = PI + wallHeading;
                        } else {
                            heading = -wallHeading;
                        }
                    }
                    case BACK -> {
                        y = wallDistance;
                        if (sensorSide == LEFT) {
                            heading = -PI / 2 + wallHeading;
                        } else {
                            heading = PI / 2 - wallHeading;
                        }
                    }
                    case LEFT -> {
                        x = wallDistance;
                        if (sensorSide == LEFT) {
                            heading = wallHeading;
                        } else {
                            heading = PI - wallHeading;
                        }
                    }
                }
                Double[] fullState;
                if (wall == Wall.FRONT || wall == Wall.BACK) {
                    fullState = new Double[]{null, y, heading};
                } else {
                    fullState = new Double[]{x, null, heading};
                }

                return fullState;
            }

            public Wall bestWall(Pose robotPose) {
                var sensor1 = c(robotPose).add(rotate(offset1, robotPose.heading));
                var sensor2 = c(robotPose).add(rotate(offset2, robotPose.heading));

                var sensorHeading = this == RIGHT ? PI - robotPose.heading : -robotPose.heading;

                var sensor1Ray = Lines.fromPointAndAngle(sensor1, sensorHeading, p).rayFrom(sensor1);
                var sensor2Ray = Lines.fromPointAndAngle(sensor2, sensorHeading, p).rayFrom(sensor2);

                if (sensor1Ray.intersection(Wall.LEFT.segment) != null && sensor2Ray.intersection(Wall.LEFT.segment) != null)
                    return Wall.LEFT;
                else if (sensor1Ray.intersection(Wall.RIGHT.segment) != null && sensor2Ray.intersection(Wall.RIGHT.segment) != null)
                    return Wall.RIGHT;
                else if (sensor1Ray.intersection(Wall.FRONT.segment) != null && sensor2Ray.intersection(Wall.FRONT.segment) != null)
                    return Wall.FRONT;
                else if (sensor1Ray.intersection(Wall.BACK.segment) != null && sensor2Ray.intersection(Wall.BACK.segment) != null)
                    return Wall.BACK;
                else
                    return null;
            }
        }

        public enum Wall {
            RIGHT(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(0, FULL_FIELD), p)),
            LEFT(Lines.segmentFromPoints(Vector2D.of(FULL_FIELD, 0), Vector2D.of(FULL_FIELD, FULL_FIELD), p)),
            BACK(Lines.segmentFromPoints(Vector2D.of(0, FULL_FIELD), Vector2D.of(FULL_FIELD, FULL_FIELD), p)),
            FRONT(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(FULL_FIELD, 0), p));

            public final Segment segment;

            Wall(Segment segment) {
                this.segment = segment;
            }
        }


        public Segment bestWall(Point relativePose, Pose robotPosition) {
            var ray = getDistanceSensorRay(relativePose, robotPosition);

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

        private Ray getDistanceSensorRay(Point relativePose, Pose robotPosition) {
            var rotatedRelative = relativePose.rotate(robotPosition.heading);
            return Lines.rayFromPointAndDirection(c(robotPosition), c(rotatedRelative), p);
        }

        public Pose getRobotPose(double distInches, double heading) {
            throw new NotImplementedError(); // TODO:  get position from dist sensor readings
        }
    }


}
