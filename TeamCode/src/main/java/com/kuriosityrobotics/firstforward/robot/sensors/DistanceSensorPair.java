package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.inRange;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.rotate;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Ray;
import org.apache.commons.geometry.euclidean.twod.Vector2D;

public enum DistanceSensorPair {
    RIGHT(
            Vector2D.of(DistanceSensorLocaliser.SENSOR_X_DISPLACEMENT_RIGHT, -DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT),
            Vector2D.of(DistanceSensorLocaliser.SENSOR_X_DISPLACEMENT_RIGHT, DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT)),
    LEFT(Vector2D.of(-DistanceSensorLocaliser.SENSOR_X_DISPLACEMENT_LEFT, -DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT),
            Vector2D.of(-DistanceSensorLocaliser.SENSOR_X_DISPLACEMENT_LEFT, DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT));

    private static final double MIN_SENSOR_DISTANCE = 1;
    private static final double MAX_SENSOR_DISTANCE = 7.75;

    private static Vector2D c(Point p) {
        return Vector2D.of(p.x, p.y);
    }

    private final Vector2D offset1, offset2;

    DistanceSensorPair(Vector2D offset1, Vector2D offset2) {
        this.offset1 = offset1;
        this.offset2 = offset2;
    }

    public static double getWallHeading(double frontSensor, double backSensor) {
        return atan2(frontSensor - backSensor, DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT);
    }

    public static double getWallDistance(double frontSensor, double backSensor) {
        double wallHeading = getWallHeading(frontSensor, backSensor);
        double d1 = frontSensor * cos(wallHeading);
        double d2 = backSensor * cos(wallHeading);
        double A = DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT / (DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT);
        double B = DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT / (DistanceSensorLocaliser.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensorLocaliser.BACK_SENSORS_Y_DISPLACEMENT);

        double d = A * d1 + B * d2;
        double dOffset = DistanceSensorLocaliser.SENSOR_X_DISPLACEMENT_RIGHT / cos(wallHeading);

        return d + dOffset;
    }

    public Double[] getPartialState(double frontSensor, double backSensor, Wall wall) {
        if (frontSensor < 0 || backSensor < 0)
            return null;

        var wallHeading = getWallHeading(frontSensor, backSensor);
        var wallDistance = getWallDistance(frontSensor, backSensor);

        double heading = 0;

        switch (wall) {
            case FRONT:
                wallDistance = FULL_FIELD - wallDistance;
                if (this == RIGHT) {
                    heading = PI / 2 + wallHeading;
                } else {
                    heading = -PI / 2 - wallHeading;
                }
                break;
            case RIGHT:
                wallDistance = FULL_FIELD - wallDistance;
                if (this == RIGHT) {
                    heading = -wallHeading;
                } else {
                    heading = PI + wallHeading;
                }
                break;
            case BACK:
                if (this == RIGHT) {
                    heading = -PI / 2 + wallHeading;
                } else {
                    heading = PI / 2 - wallHeading;
                }
                break;
            case LEFT:
                if (this == LEFT) {
                    heading = wallHeading;
                } else {
                    heading = PI - wallHeading;
                }
                break;
        }

        Double[] pose;
        if (wall == Wall.FRONT || wall == Wall.BACK)
            pose = new Double[]{null, wallDistance, heading};
        else
            pose = new Double[]{wallDistance, null, heading};

        return pose;
    }

    public Wall bestWall(Pose covariance, Pose robotPose) {
        var sensor1 = c(robotPose).add(rotate(offset1, robotPose.heading));
        var sensor2 = c(robotPose).add(rotate(offset2, robotPose.heading));

        var sensorHeading = PI/2 + (this == RIGHT ? -(robotPose.heading + PI/2) : -(robotPose.heading - PI/2));

        var sensor1Line = Lines.fromPointAndAngle(sensor1, sensorHeading, DistanceSensorLocaliser.p);
        var sensor1Segment = sensor1Line.rayFrom(sensor1);

        var sensor2Line = Lines.fromPointAndAngle(sensor2, sensorHeading, DistanceSensorLocaliser.p);
        var sensor2Segment = sensor2Line.rayFrom(sensor2);

        if (wallInRangeOfRays(covariance, Wall.LEFT, sensor1Segment, sensor2Segment))
            return Wall.LEFT;
        else if (wallInRangeOfRays(covariance, Wall.RIGHT, sensor1Segment, sensor2Segment))
            return Wall.RIGHT;
        else if (wallInRangeOfRays(covariance, Wall.FRONT, sensor1Segment, sensor2Segment))
            return Wall.FRONT;
        else if (wallInRangeOfRays(covariance, Wall.BACK, sensor1Segment, sensor2Segment))
            return Wall.BACK;
        else
            return null;
    }

    private boolean wallInRangeOfRays(Pose variance, Wall wall, Ray... rays) {
        for (var sensorRay : rays) {
            var wallIntersection = sensorRay.intersection(wall.getSegment());
            if (wallIntersection == null)
                return false;

            var wallDistance = sensorRay.getStartPoint().distance(wallIntersection);
            if (!inRange(MIN_SENSOR_DISTANCE - variance.norm(), MAX_SENSOR_DISTANCE + variance.norm(), wallDistance))
                return false;
        }

        return true;
    }
}
