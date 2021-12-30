package com.kuriosityrobotics.firstforward.robot.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Constants {
    public static final double EPSILON = 0.1;

    public static class Dashboard {
        public static final double ROBOT_RADIUS = 6; // in
    }

    public static class Webcam {
        public static final float MM_PER_INCH = 25.4f;

        // Camera positions on robot (both front and left)
        // it is correct but vuforia sucks when it is too close to the wall target(2-3 inches off)
        public static final float CAMERA_LEFT_FORWARD_DISPLACEMENT = .125f * MM_PER_INCH;
        public static final float CAMERA_LEFT_VERTICAL_DISPLACEMENT = 7.25f * MM_PER_INCH;
        public static final float CAMERA_LEFT_LEFT_DISPLACEMENT = 5.5f * MM_PER_INCH;

        public static final float CAMERA_FRONT_FORWARD_DISPLACEMENT = 8.075f * MM_PER_INCH;
        public static final float CAMERA_FRONT_VERTICAL_DISPLACEMENT = 15.313f * MM_PER_INCH;
        public static final float CAMERA_FRONT_LEFT_DISPLACEMENT = 0.185f * MM_PER_INCH;

        // Constants for perimeter targets
        public static final float MM_TARGET_HEIGHT = 6f * MM_PER_INCH;
        public static final float HALF_FIELD = 70f * MM_PER_INCH;
        public static final float ONE_TILE = 23.5f * MM_PER_INCH;
        public static final float FULL_FIELD = HALF_FIELD * 2f;
        public static final float ONE_AND_HALF_TILE = ONE_TILE * 1.5f;
        public static final float HALF_TILE = ONE_TILE * 0.5f;

        public static final double HALF_ROBOT_WIDTH = 11.75 / 2;
        public static final double HALF_ROBOT_LENGTH = 12.75 / 2;

        public static final OpenGLMatrix CAMERA_LEFT_LOCATION_ON_ROBOT = OpenGLMatrix
                .translation(CAMERA_LEFT_FORWARD_DISPLACEMENT, CAMERA_LEFT_LEFT_DISPLACEMENT, CAMERA_LEFT_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 180, 0));

        public static final OpenGLMatrix CAMERA_FRONT_LOCATION_ON_ROBOT = OpenGLMatrix
                .translation(CAMERA_FRONT_FORWARD_DISPLACEMENT, CAMERA_FRONT_LEFT_DISPLACEMENT, CAMERA_FRONT_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 30));
    }
}
