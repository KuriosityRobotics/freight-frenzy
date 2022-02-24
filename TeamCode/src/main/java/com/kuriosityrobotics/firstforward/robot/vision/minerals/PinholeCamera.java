package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class PinholeCamera {
    private final Rotation cameraRotation;
    private final Vector3D cameraPosition;

    // in pixels (yes, focal length is in px)
    private final double fx, fy, originX, originY, width, height;

    // in mm (diagonal;  used with hypot(width, height) to convert between pixels and coords)
    private final double sensorDiagonalUnits;

    public PinholeCamera(double fx, double fy, double originX, double originY, double width, double height,
                         double sensorDiagonalUnits, Rotation cameraRotation, Vector3D cameraPosition) {
        this.originX = originX;
        this.originY = originY;
        this.width = width;
        this.height = height;
        this.sensorDiagonalUnits = sensorDiagonalUnits;
        this.fx = fx;
        this.fy = fy;
        this.cameraRotation = cameraRotation;
        this.cameraPosition = cameraPosition;

        if (cameraPosition != null && cameraPosition.getY() == 0)
            throw new IllegalArgumentException("camera position y-coordinate musn't be 0");
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
        rayUnits = rayUnits.scalarMultiply(1 / (rayUnits.getEntry(2, 0))); // we want Z to be 1 unit

        var vec = cameraRotation.applyInverseTo(new Vector3D(rayUnits.getColumn(0)));
        if (cameraPosition != null) {
            vec = vec.scalarMultiply(-cameraPosition.getY() / vec.getY()); // we want y to equal zero.
            vec = vec.add(cameraPosition);
        }
        return vec;
    }

}
