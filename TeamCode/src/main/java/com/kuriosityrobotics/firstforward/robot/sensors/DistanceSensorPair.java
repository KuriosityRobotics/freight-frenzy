package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.rotate;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Vector2D;

public enum DistanceSensorPair {
    RIGHT(
            Vector2D.of(-DistanceSensors.SENSOR_X_DISPLACEMENT, -DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT),
            Vector2D.of(-DistanceSensors.SENSOR_X_DISPLACEMENT, DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT)),
    LEFT(Vector2D.of(DistanceSensors.SENSOR_X_DISPLACEMENT, -DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT),
            Vector2D.of(DistanceSensors.SENSOR_X_DISPLACEMENT, DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT));

    private static Vector2D c(Point p) {
        return Vector2D.of(p.x, p.y);
    }

    private final Vector2D offset1, offset2;

    DistanceSensorPair(Vector2D offset1, Vector2D offset2) {
        this.offset1 = offset1;
        this.offset2 = offset2;
    }

    public static double getWallHeading(double frontSensor, double backSensor) {
        return atan2(frontSensor - backSensor, DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT);
    }

    public static double getWallDistance(double frontSensor, double backSensor) {
        double wallHeading = getWallHeading(frontSensor, backSensor);
        double d1 = frontSensor * cos(wallHeading);
        double d2 = backSensor * cos(wallHeading);
        double A = DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT / (DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT);
        double B = DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT / (DistanceSensors.FRONT_SENSORS_Y_DISPLACEMENT + DistanceSensors.BACK_SENSORS_Y_DISPLACEMENT);

        double d = A * d1 + B * d2;
        double dOffset = DistanceSensors.SENSOR_X_DISPLACEMENT / cos(wallHeading);

        return d + dOffset;
    }

    public Double[] getPartialState(double frontSensor, double backSensor, DistanceSensors.Wall wall) {
        var wallHeading = getWallHeading(frontSensor, backSensor);
        var wallDistance = getWallDistance(frontSensor, backSensor);

        double heading = 0;

        switch (wall) {
            case FRONT:
                wallDistance = FULL_FIELD - wallDistance;
                if (this == LEFT) {
                    heading = PI / 2 + wallHeading;
                } else {
                    heading = -PI / 2 - wallHeading;
                }
                break;
            case RIGHT:
                wallDistance = FULL_FIELD - wallDistance;
                if (this == LEFT) {
                    heading = PI + wallHeading;
                } else {
                    heading = -wallHeading;
                }
                break;
            case BACK:
                if (this == LEFT) {
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
        if (wall == DistanceSensors.Wall.FRONT || wall == DistanceSensors.Wall.BACK)
            pose = new Double[]{null, wallDistance, heading};
        else
            pose = new Double[]{wallDistance, null, heading};

        return pose;
    }

    public DistanceSensors.Wall bestWall(Pose robotPose) {
        var sensor1 = c(robotPose).add(rotate(offset1, robotPose.heading));
        var sensor2 = c(robotPose).add(rotate(offset2, robotPose.heading));

        var sensorHeading = this == RIGHT ? PI - robotPose.heading : -robotPose.heading;

        var sensor1Line = Lines.fromPointAndAngle(sensor1, sensorHeading, DistanceSensors.p);
        var sensor1Segment = sensor1Line.segment(sensor1, sensor1.add(sensor1Line.getDirection().multiply(10)));

        var sensor2Line = Lines.fromPointAndAngle(sensor2, sensorHeading, DistanceSensors.p);
        var sensor2Segment = sensor2Line.segment(sensor2, sensor2.add(sensor2Line.getDirection().multiply(10)));

        if (sensor1Segment.intersection(DistanceSensors.Wall.LEFT.getSegment()) != null && sensor2Segment.intersection(DistanceSensors.Wall.LEFT.getSegment()) != null)
            return DistanceSensors.Wall.LEFT;
        else if (sensor1Segment.intersection(DistanceSensors.Wall.RIGHT.getSegment()) != null && sensor2Segment.intersection(DistanceSensors.Wall.RIGHT.getSegment()) != null)
            return DistanceSensors.Wall.RIGHT;
        else if (sensor1Segment.intersection(DistanceSensors.Wall.FRONT.getSegment()) != null && sensor2Segment.intersection(DistanceSensors.Wall.FRONT.getSegment()) != null)
            return DistanceSensors.Wall.FRONT;
        else if (sensor1Segment.intersection(DistanceSensors.Wall.BACK.getSegment()) != null && sensor2Segment.intersection(DistanceSensors.Wall.BACK.getSegment()) != null)
            return DistanceSensors.Wall.BACK;
        else
            return null;
    }
}
