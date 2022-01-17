package com.kuriosityrobotics.firstforward.robot.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Constants {
    public static class OpModes {
        public static final double JOYSTICK_EPSILON = 0.1;
    }

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

        public static final String VUFORIA_LICENCE_KEY =
                "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
                        "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
                        "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
                        "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
                        "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
                        "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
                        "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";
    }

    public static class Detect {
        public static final Vector3D RED = new Vector3D(255, 0, 0);
    }
}
