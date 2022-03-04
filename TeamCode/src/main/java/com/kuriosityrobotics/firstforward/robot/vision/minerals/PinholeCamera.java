package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;

import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class PinholeCamera {
    private final PhysicalCamera physicalCamera;

    // in pixels (yes, focal length is in px)
    private final double fx, fy, originX, originY, width, height;

    // in mm (diagonal;  used with hypot(width, height) to convert between pixels and coords)
    private final double sensorDiagonalUnits;

    public PinholeCamera(double fx, double fy, double originX, double originY, double width, double height,
                         double sensorDiagonalUnits, PhysicalCamera physicalCamera) {
        this.originX = originX;
        this.originY = originY;
        this.width = width;
        this.height = height;
        this.sensorDiagonalUnits = sensorDiagonalUnits;
        this.fx = fx;
        this.fy = fy;
        this.physicalCamera = physicalCamera;
    }

    private static RealMatrix framePixelCoordsToMatrix(double u, double v) {
        return MatrixUtils.createColumnRealMatrix(new double[]{u, v, 1});
    }

    private double pxToUnits(double pixels) {
        return pixels * sensorDiagonalUnits / Math.hypot(width, height);
    }

    private double unitsToPx(double units) {
        return units * Math.hypot(width, height) / sensorDiagonalUnits;
    }

    private Vector3D unitsToPx(Vector3D units) {
        return units.scalarMultiply(Math.hypot(width, height) / sensorDiagonalUnits);
    }

    private RealMatrix pxToUnits(RealMatrix pixels) {
        return pixels.scalarMultiply(sensorDiagonalUnits / Math.hypot(width, height));
    }

    private RealMatrix normaliseFrameCoordinates() {
        return MatrixUtils.createRealMatrix(new double[][]
                {
                        {1, 0, -originX},
                        {0, -1, originY},
                        {0, 0, 1}
                });
    }

    private RealMatrix normalisedFrameTo3DPixel() {
        return MatrixUtils.createRealMatrix(new double[][]
                {
                        {1 / fx, 0, 0},
                        {0, 1 / fy, 0},
                        {0, 0, 1}
                });
    }

    public Vector3D unprojectFramePixelsToRay(double u, double v) {
        var rayPixels =
                normalisedFrameTo3DPixel().multiply(normaliseFrameCoordinates()).multiply(framePixelCoordsToMatrix(u, v));
        var rayUnits = pxToUnits(rayPixels);

        var vec = physicalCamera.robotToCameraRotation().applyInverseTo(new Vector3D(rayUnits.getColumn(0)));
        var field = new Plane(new Vector3D(0, 1, 0));
        var line = new Line(physicalCamera.robotToCameraTranslation(), physicalCamera.robotToCameraTranslation().add(vec.normalize()));
        return field.intersection(line)
                .add(physicalCamera.robotLocationProvider().getTranslation());
    }

    public Vector2D getLocationOnFrame(Vector3D position) {
        position = position.subtract(physicalCamera.robotLocationProvider().getTranslation());
        position = position.subtract(physicalCamera.robotToCameraTranslation());
        position = physicalCamera.robotToCameraRotation().applyTo(position);
        return new Vector2D(position.getX() * (fx / position.getZ()), position.getY() * (fy / position.getZ()));
    }


}
