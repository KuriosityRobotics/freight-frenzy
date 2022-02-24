package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VARIABLE_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VERTICAL_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_VERTICAL_DISPLACEMENT_MM;
import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.Robot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

class RobotCamera extends PhysicalCamera {
    private final Robot robot;

    protected RobotCamera(Robot robot) {
        this.robot = robot;
    }

    private float cameraAngle() {
        if(robot.visionThread == null)
            return 0f;
        else
            return (float) robot.visionThread.getCameraAngle();
    }

    @Override
    public Vector3D robotToCameraTranslation() {
        return new Vector3D(
                SERVO_FORWARD_DISPLACEMENT_MM + CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.cos(cameraAngle()),
                SERVO_LEFT_DISPLACEMENT_MM - CAMERA_VARIABLE_DISPLACEMENT_MM * (float)Math.sin(cameraAngle()),
                SERVO_VERTICAL_DISPLACEMENT_MM + CAMERA_VERTICAL_DISPLACEMENT_MM
        ).scalarMultiply(1/MM_PER_INCH);
    }

    @Override
    public Rotation robotToCameraRotation() {
        return new Rotation(RotationOrder.XZY, RotationConvention.VECTOR_OPERATOR,  Math.toRadians(33), cameraAngle(), 0);
    }
}
