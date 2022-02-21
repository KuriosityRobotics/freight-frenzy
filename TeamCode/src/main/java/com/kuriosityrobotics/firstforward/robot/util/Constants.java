package com.kuriosityrobotics.firstforward.robot.util;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

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
        public static final float SERVO_FORWARD_DISPLACEMENT_MM = 4.821f * MM_PER_INCH;
        public static final float SERVO_VERTICAL_DISPLACEMENT_MM = 16.519f * MM_PER_INCH;
        public static final float SERVO_LEFT_DISPLACEMENT_MM = 0.318f * MM_PER_INCH;

        // camera pos relative to the turret servo
        public static final float CAMERA_VARIABLE_DISPLACEMENT_MM = 3.382f * MM_PER_INCH;
        public static final float CAMERA_VERTICAL_DISPLACEMENT_MM = 0f * MM_PER_INCH;

        // Constants for perimeter targets
        public static final float TARGET_HEIGHT_MM = 6f * MM_PER_INCH;
        public static final float HALF_FIELD_MM = 70.5f * MM_PER_INCH;
        public static final float ONE_TILE_MM = 23.375f * MM_PER_INCH;
        public static final float FULL_FIELD_MM = HALF_FIELD_MM * 2f;
        public static final float ONE_AND_HALF_TILE_MM = 35f * MM_PER_INCH;
        public static final float HALF_TILE_MM = 11.625f * MM_PER_INCH;

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
