package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VARIABLE_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VERTICAL_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.*;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_VERTICAL_DISPLACEMENT_MM;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

class RobotCamera extends PhysicalCamera {

    private final Robot robot;

    protected RobotCamera(Robot robot) {
        this.robot = robot;
    }

    public float robotRelativeCameraAngle() {
        if(robot.visionThread == null)
            return 0f;
        else
            return (float) robot.visionThread.getCameraAngle();
    }

    @Override
    public Vector3D robotToCameraTranslation() {
        return new Vector3D(
                -(SERVO_LEFT_DISPLACEMENT_MM - CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.sin(robotRelativeCameraAngle())),
                SERVO_VERTICAL_DISPLACEMENT_MM + CAMERA_VERTICAL_DISPLACEMENT_MM,
                SERVO_FORWARD_DISPLACEMENT_MM + CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.cos(robotRelativeCameraAngle())
                ).scalarMultiply(1/MM_PER_INCH);
    }

    @Override
    public Rotation robotToCameraRotation() {
        return new Rotation(new Vector3D(1, 0, 0), Math.PI/6).applyTo(new Rotation(new Vector3D(0, 1, 0), -(robotRelativeCameraAngle() + robot.getPose().heading)));
    }

    @Override
    public LocationProvider robotLocationProvider() {
        return robot;
    }
}
