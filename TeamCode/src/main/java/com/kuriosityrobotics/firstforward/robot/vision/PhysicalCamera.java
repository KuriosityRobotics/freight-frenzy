package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.PinholeCamera;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class PhysicalCamera {
    /*
                 size="800, 448"
            focalLength="578.272f , 578.272f"
            principalPoint="402.145f , 221.506f"
            distortionCoefficients="0.12175, -0.251652 , 0, 0, 0.112142, 0, 0, 0"
            />
     */
    public static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    public static final double FRAME_WIDTH = 800;
    public static final double FOCAL_LENGTH_X = 578.272;
    public static final double O_X = 402.145;

    public static final double FRAME_HEIGHT = 448;
    public static final double FOCAL_LENGTH_Y = 578.272 ;
    public static final double O_Y = 221.506;

    public LocationProvider robotLocationProvider() {
        return LocationProvider.of(Pose.ZERO, Pose.ZERO);
    }
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

    public PinholeCamera pinholeCamera() {
        return new PinholeCamera(
                PhysicalCamera.FOCAL_LENGTH_X,
                PhysicalCamera.FOCAL_LENGTH_Y,
                PhysicalCamera.O_X,
                PhysicalCamera.O_Y,
                PhysicalCamera.FRAME_WIDTH,
                PhysicalCamera.FRAME_HEIGHT,
                PhysicalCamera.SENSOR_DIAGONAL,
                this
        );
    }
}
