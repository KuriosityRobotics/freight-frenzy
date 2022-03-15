package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import static org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D.createRotation;
import static org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D.createScale;
import static org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D.createTranslation;
import static java.lang.Math.PI;

import org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D;
import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.geometry.euclidean.twod.AffineTransformMatrix2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;

public class PinholeCamera {


    private static final Vector3D robotToCameraTranslation =
            Vector3D.of(0, 16.404, 4.821);
    private static final AffineTransformMatrix3D robotToCameraRotation = AffineTransformMatrix3D.createRotation(Vector3D.ZERO
            , QuaternionRotation.fromAxisAngle(Vector3D.of(-1, 0, 0), PI / 6));
    private static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    private static final double FRAME_WIDTH = 800;
    private static final double FOCAL_LENGTH_X = 578.272;
    private static final double O_X = 402.145;

    private static final double FRAME_HEIGHT = 448;
    private static final double O_Y = 221.506;
    // in pixels
    private final double focalLength, originX, originY, width, height;
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
        return getZPrimeConstrainedColumn(worldToFrame, 2, Z, x, y);
    }

    private AffineTransformMatrix2D normaliseFrameCoordinates() { // TODO:  stop abusing the word 'normalise'
        return AffineTransformMatrix2D.createScale(1, -1).translate(-originX, originY);
    }

    AffineTransformMatrix3D cameraMatrix(Vector3D robotPosition, double heading) {
        var fieldToRobotRotation = createRotation(Vector3D.ZERO, QuaternionRotation.fromAxisAngle(Vector3D.of(0, 1, 0), -heading));
        return createTranslation(robotPosition.add(fieldToRobotRotation.apply(robotToCameraTranslation)).negate())
                .premultiply(robotToCameraRotation.multiply(fieldToRobotRotation))
                .premultiply(createScale(Math.hypot(width, height) / sensorDiagonalUnits))
                .premultiply(createScale(focalLength, focalLength, 1));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double heading, double u, double v) {
        var normalisedCoordinates = Vector2D.of(u, v).transform(normaliseFrameCoordinates());
        double x = normalisedCoordinates.getX(), y = normalisedCoordinates.getY();

        double zPrime = getZPrimeConstrainedY(cameraMatrix(robotPosition, heading), 0, x, y);
        double xPrime = x * zPrime, yPrime = y * zPrime;

        return cameraMatrix(robotPosition, heading).inverse().apply(Vector3D.of(xPrime, yPrime, zPrime));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double heading, Vector2D locationOnFrame) {
        return getLocationOnField(robotPosition, heading, locationOnFrame.getX(), locationOnFrame.getY());
    }

    public Vector2D getLocationOnFrame(Vector3D robotPosition, double heading, Vector3D position) {
        var transformed = position.transform(cameraMatrix(robotPosition, heading));

        var res = transformed.multiply(1 / transformed.getZ());
        return Vector2D.of(res.getX(), res.getY()).transform(normaliseFrameCoordinates().inverse());
    }

    public final PositionBoundPinholeCamera bindToPose(Vector3D robotPosition, double heading) {
        return this.new PositionBoundPinholeCamera(robotPosition, heading);
    }

    public class PositionBoundPinholeCamera {
        public final Vector3D robotPosition;
        public final double heading;

        PositionBoundPinholeCamera(Vector3D robotPosition, double heading) {
            this.robotPosition = robotPosition;
            this.heading = heading;
        }

        public Vector2D getLocationOnFrame(Vector3D position) {
            return PinholeCamera.this.getLocationOnFrame(robotPosition, heading, position);
        }

        public Vector3D getLocationOnField(double u, double v) {
            return PinholeCamera.this.getLocationOnField(robotPosition, heading, u, v);
        }

        public Vector3D getLocationOnField(Vector2D point) {
            return PinholeCamera.this.getLocationOnField(robotPosition, heading, point);
        }

    }
}
