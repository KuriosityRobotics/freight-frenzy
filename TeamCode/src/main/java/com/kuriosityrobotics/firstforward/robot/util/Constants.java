package com.kuriosityrobotics.firstforward.robot.util;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.MM_PER_INCH;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * All units are in inches and radians unless otherwise specified.
 */
public class Constants {

    public static class OpModes {
        public static final double JOYSTICK_EPSILON = 0.1;
    }

    public static class Dashboard {
        public static final double ROBOT_RADIUS = 6; // in
    }

    public static final class Units {
        public static final float MM_PER_INCH = 25.4f;
    }

    public static class Field {
        // field constants
        public static final float TILE_MEAT = 23f;
        public static final float TILE_TAB = 0.5f;

//        public static final float FULL_FIELD = (6 * TILE_MEAT) + (5 * TILE_TAB);
        public static final float FULL_FIELD = 141.375f;

        public static final float TILE_MEAT_MM = TILE_MEAT * MM_PER_INCH;
        public static final float HALF_TILE_MEAT_MM = TILE_MEAT_MM * 0.5f;
        public static final float TILE_TAB_MM = TILE_TAB * MM_PER_INCH;

        public static final float FULL_FIELD_MM = FULL_FIELD * MM_PER_INCH;
        public static final float HALF_FIELD_MM = FULL_FIELD_MM * 0.5f;

        // Constants for perimeter targets
        public static final float TARGET_HEIGHT_MM = 5.75f * MM_PER_INCH;

        // Hub Positions
        public static final float RED_HUB_X = 2f * TILE_MEAT + 1.5f * TILE_TAB;
        public static final float RED_HUB_Y = 3.5f * TILE_MEAT + 3f * TILE_TAB;

        public static final float BLUE_HUB_X = FULL_FIELD - RED_HUB_X;
        public static final float BLUE_HUB_Y = RED_HUB_Y;

        public static final float SHARE_HUB_X = FULL_FIELD / 2.0f;
        public static final float SHARE_HUB_Y = TILE_MEAT / 2.0f;

        public static final ArrayList<Point> HUBS = new ArrayList<>(Arrays.asList(
                new Point(RED_HUB_X, RED_HUB_Y),
                new Point(BLUE_HUB_X, BLUE_HUB_Y)));
    }

    public static class Webcam {

        // Camera positions on robot (both front and left)
        // it is correct but vuforia sucks when it is too close to the wall target(2-3 inches off)
        public static final float SERVO_FORWARD_DISPLACEMENT_MM = 4.821f * MM_PER_INCH;
        public static final float SERVO_VERTICAL_DISPLACEMENT_MM = 16.404f * MM_PER_INCH;
        public static final float SERVO_LEFT_DISPLACEMENT_MM = 0.318f * MM_PER_INCH;

        // camera pos relative to the turret servo
        public static final float CAMERA_VARIABLE_DISPLACEMENT_MM = 3.365f * MM_PER_INCH;
        public static final float CAMERA_VERTICAL_DISPLACEMENT_MM = 0f * MM_PER_INCH;

        public static final String VUFORIA_LICENCE_KEY =
                "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
                        "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
                        "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
                        "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
                        "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
                        "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
                        "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";
    }
}
