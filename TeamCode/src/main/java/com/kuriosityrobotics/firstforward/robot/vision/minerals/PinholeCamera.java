package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D;
import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.geometry.euclidean.twod.AffineTransformMatrix2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;

import static java.lang.Math.*;
import static org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D.*;

public class PinholeCamera {


    private static final Vector3D robotToCameraTranslationBase =
            Vector3D.of(0.3, 16.404, 4.821);
    private static final AffineTransformMatrix3D robotToCameraRotation = AffineTransformMatrix3D.createRotation(Vector3D.ZERO
            , QuaternionRotation.fromAxisAngle(Vector3D.of(1, 0, 0), PI / 6));
    private static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    private static final double FRAME_WIDTH = 800;
    private static final double FOCAL_LENGTH_X = 578.272;
    private static final double O_X = 402.145;

    private static final double FRAME_HEIGHT = 448;
    private static final double O_Y = 221.506;
    // in pixels
    private double focalLength, originX, originY, width, height;
    // in units (diagonal;  used with hypot(width, height) to convert between pixels and coords)
    private final double sensorDiagonalUnits;

    private PinholeCamera(double focalLength, double originX, double originY, double width, double height,
                          double sensorDiagonalUnits) {
        this.originX = originX;
        this.originY = originY;
        this.width = width;
        this.height = height;
        this.sensorDiagonalUnits = sensorDiagonalUnits;
        this.focalLength = focalLength;
    }

    public static PinholeCamera create() {
        return new PinholeCamera(
                FOCAL_LENGTH_X,
                O_X,
                O_Y,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                SENSOR_DIAGONAL);
    }

    /**
     * Find z' given the frame coordinates and constrained world coordinate.
     *
     * @param worldToFrame
     * @param x
     * @param y
     * @return
     */
    private static double getZPrimeConstrainedColumn(AffineTransformMatrix3D worldToFrame, int column, double constraint, double x, double y) {
        var frameToWorld = worldToFrame.inverse().toArray();
        int start = column * 4;
        double e = frameToWorld[start], f = frameToWorld[start + 1], g = frameToWorld[start + 2], h = frameToWorld[start + 3];
        return (constraint - h) / (e * x + f * y + g);
    }

    private static double getZPrimeConstrainedX(AffineTransformMatrix3D worldToFrame, double X, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 0, X, x, y);
    }

    private static double getZPrimeConstrainedY(AffineTransformMatrix3D worldToFrame, double Y, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 1, Y, x, y);
    }

    private static double getZPrimeConstrainedZ(AffineTransformMatrix3D worldToFrame, double Z, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 1, Z, x, y);
    }

    private AffineTransformMatrix2D normaliseFrameCoordinates() { // TODO:  stop abusing the word 'normalise'
        return AffineTransformMatrix2D.createScale(1, -1).translate(-originX, originY);
    }

    AffineTransformMatrix3D cameraMatrix(Vector3D robotPosition, double cameraAngle, double globalHeading) {
        var variableRotation = Vector3D.of(2.5f * sin(cameraAngle), 0, 2.5f * cos(cameraAngle)); // <0, -3.365>
        var fieldToRobotRotation = createRotation(Vector3D.ZERO, QuaternionRotation.fromAxisAngle(Vector3D.of(0, 1, 0), (globalHeading)));

        var translation = robotToCameraTranslationBase.add(variableRotation);
        translation = fieldToRobotRotation.apply(translation);
        return (robotToCameraRotation).multiply(fieldToRobotRotation)
                .premultiply(createTranslation(robotPosition.add(translation).negate()))
//                .premultiply()
                .premultiply(createScale(Math.hypot(width, height) / sensorDiagonalUnits))
                .premultiply(createScale(focalLength, focalLength, 1));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, double u, double v) {
        var normalisedCoordinates = Vector2D.of(u, v).transform(normaliseFrameCoordinates());
        double x = normalisedCoordinates.getX(), y = normalisedCoordinates.getY();

        double zPrime = getZPrimeConstrainedY(cameraMatrix(robotPosition, cameraAngle, globalHeading), 0, x, y);
        double xPrime = x * zPrime, yPrime = y * zPrime;

        return cameraMatrix(robotPosition, cameraAngle, globalHeading).inverse().apply(Vector3D.of(xPrime, yPrime, zPrime));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, Vector2D locationOnFrame) {
        return getLocationOnField(robotPosition, cameraAngle, globalHeading, locationOnFrame.getX(), locationOnFrame.getY());
    }

    public Vector2D getLocationOnFrame(Vector3D robotPosition, double cameraAngle, double globalHeading, Vector3D position) {
        var transformed = position.transform(cameraMatrix(robotPosition, cameraAngle, globalHeading));

        var res = transformed.multiply(1 / transformed.getZ());
        return Vector2D.of(res.getX(), res.getY()).transform(normaliseFrameCoordinates().inverse());
    }

    public final PositionBoundPinholeCamera bindToPose(Vector3D robotPosition, double cameraAngle, double globalHeading) {
        return this.new PositionBoundPinholeCamera(robotPosition, cameraAngle, globalHeading);
    }

    public class PositionBoundPinholeCamera {
        public final Vector3D robotPosition;
        public final double cameraAngle, globalHeading;

        PositionBoundPinholeCamera(Vector3D robotPosition, double cameraAngle, double globalHeading) {
            this.robotPosition = robotPosition;
            this.cameraAngle = cameraAngle;
            this.globalHeading = globalHeading;
        }

        public Vector2D getLocationOnFrame(Vector3D position) {
            return PinholeCamera.this.getLocationOnFrame(robotPosition, cameraAngle, globalHeading, position);
        }

        public Vector3D getLocationOnField(double u, double v) {
            return PinholeCamera.this.getLocationOnField(robotPosition, cameraAngle, globalHeading, u, v);
        }

        public Vector3D getLocationOnField(Vector2D point) {
            return PinholeCamera.this.getLocationOnField(robotPosition, cameraAngle, globalHeading, point);
        }

    }
}
