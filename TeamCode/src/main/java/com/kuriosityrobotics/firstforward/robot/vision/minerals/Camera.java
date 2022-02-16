package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class Camera {
    private final Rotation camera_rotation;
    private final Vector3D camera_position;

    // in pixels (yes, focal length is in px)
    private final double focal_length, origin_x, origin_y, width, height;

    // in mm (diagonal;  used with hypot(width, height) to convert between pixels and coords)
    private final double sensor_diagonal_units;

    public Camera(double focal_length_px, double origin_x_px, double origin_y_px, double width_px, double height_px,
                  double sensor_diagonal_units) {
        this(focal_length_px, origin_x_px, origin_y_px, width_px, height_px, sensor_diagonal_units,
                Rotation.IDENTITY, null);
    }

    public Camera(double focal_length_px, double origin_x_px, double origin_y_px, double width_px, double height_px,
                  double sensor_diagonal_units, Rotation camera_rotation) {
        this(focal_length_px, origin_x_px, origin_y_px, width_px, height_px, sensor_diagonal_units,
                camera_rotation, null);
    }

    public Camera(double focal_length_px, double origin_x_px, double origin_y_px, double width_px, double height_px,
                  double sensor_diagonal_units, Vector3D camera_position) {
        this(focal_length_px, origin_x_px, origin_y_px, width_px, height_px, sensor_diagonal_units,
                Rotation.IDENTITY, camera_position);
    }

    public Camera(double focal_length_px, double origin_x_px, double origin_y_px, double width_px, double height_px,
                  double sensor_diagonal_units, Rotation camera_rotation, Vector3D camera_position) {
        this.origin_x = origin_x_px;
        this.origin_y = origin_y_px;
        this.width = width_px;
        this.height = height_px;
        this.sensor_diagonal_units = sensor_diagonal_units;
        this.focal_length = focal_length_px;
        this.camera_rotation = camera_rotation;
        this.camera_position = camera_position;

        if (camera_position != null && camera_position.getY() == 0)
            throw new IllegalArgumentException("camera position y-coordinate musn't be 0");
    }

    private static RealMatrix framePixelCoordsToMatrix(double u, double v) {
        return MatrixUtils.createColumnRealMatrix(new double[]{u, v, 1});
    }

    private double pxToUnits(double pixels) {
        return pixels * sensor_diagonal_units / Math.hypot(width, height);
    }

    private double unitsToPx(double units) {
        return units * Math.hypot(width, height) / sensor_diagonal_units;
    }

    private RealMatrix pxToUnits(RealMatrix pixels) {
        return pixels.scalarMultiply(sensor_diagonal_units / Math.hypot(width, height));
    }

    private RealMatrix normaliseFrameCoordinates() {
        return MatrixUtils.createRealMatrix(new double[][]
                {
                        {1, 0, -origin_x},
                        {0, -1, origin_y},
                        {0, 0, 1}
                });
    }

    private RealMatrix normalisedFrameTo3DPixel() {
        return MatrixUtils.createRealMatrix(new double[][]
                {
                        {1 / focal_length, 0, 0},
                        {0, 1 / focal_length, 0},
                        {0, 0, 1}
                });
    }

    public Vector3D unprojectFramePixelsToRay(double u, double v) {
        var ray_pixels =
                normalisedFrameTo3DPixel().multiply(normaliseFrameCoordinates()).multiply(framePixelCoordsToMatrix(u,
                        v));
        var ray_units = pxToUnits(ray_pixels);
        ray_units = ray_units.scalarMultiply(1 / (ray_units.getEntry(2, 0))); // we want Z to be 1 unit

        var vec = camera_rotation.applyInverseTo(new Vector3D(ray_units.getColumn(0)));
        if (camera_position != null) {
            vec = vec.scalarMultiply(-camera_position.getY() / vec.getY()); // we want y to equal zero.
            vec = vec.add(camera_position);
        }
        return vec;
    }

}
