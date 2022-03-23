package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VARIABLE_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VERTICAL_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.*;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_VERTICAL_DISPLACEMENT_MM;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.geometry.euclidean.threed.rotation.Rotation3D;

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
        return Vector3D.of(
                -(SERVO_LEFT_DISPLACEMENT_MM - CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.sin(robotRelativeCameraAngle())),
                SERVO_VERTICAL_DISPLACEMENT_MM + CAMERA_VERTICAL_DISPLACEMENT_MM,
                SERVO_FORWARD_DISPLACEMENT_MM + CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.cos(robotRelativeCameraAngle())
                ).multiply(1/MM_PER_INCH);
    }

    @Override
    public Rotation3D robotToCameraRotation() {
        return QuaternionRotation.fromAxisAngle(Vector3D.of(1, 0, 0), Math.PI/6).multiply(QuaternionRotation.fromAxisAngle(Vector3D.of(0, 1, 0), -(robotRelativeCameraAngle() + robot.getPose().heading)));
    }

    @Override
    public LocationProvider robotLocationProvider() {
        return robot;
    }
}
