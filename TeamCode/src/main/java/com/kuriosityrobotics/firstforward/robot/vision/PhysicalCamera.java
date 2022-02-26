package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.Robot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class PhysicalCamera {
    public abstract Vector3D robotToCameraTranslation();
    public abstract Rotation robotToCameraRotation();

    public OpenGLMatrix rotationMatrix() {
        var vec = robotToCameraRotation().getAxis(RotationConvention.VECTOR_OPERATOR);
        return OpenGLMatrix.rotation(RADIANS, (float) robotToCameraRotation().getAngle(), (float)vec.getX(), (float)vec.getY(), (float)vec.getZ())
                .rotated(AxesReference.INTRINSIC, AxesOrder.XZY, RADIANS, (float)PI/2, (float)PI/2, 0);
    }

    public OpenGLMatrix translationMatrix() {
        Vector3D translationVector = robotToCameraTranslation();
        return OpenGLMatrix.translation(
                (float) translationVector.getX(),
                (float) translationVector.getY(),
                (float) translationVector.getZ()
        ).scaled(MM_PER_INCH);
    }

    public static PhysicalCamera of(Vector3D translation, Rotation rotation) {
        return new PhysicalCamera() {
            @Override
            public Vector3D robotToCameraTranslation() {
                return translation;
            }

            @Override
            public Rotation robotToCameraRotation() {
                return rotation;
            }
        };
    }

    public static RobotCamera of(Robot robot) {
        return new RobotCamera(robot);
    }
}
