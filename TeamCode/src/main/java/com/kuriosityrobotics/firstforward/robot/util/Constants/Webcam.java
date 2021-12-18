package com.kuriosityrobotics.firstforward.robot.util.Constants;

public class Webcam {
    public static final float MM_PER_INCH = 25.4f;

    // current pos matches tuning, not supposed to match actual pos on the robot
    public static final float CAMERA_FORWARD_DISPLACEMENT = 5.375f * MM_PER_INCH;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 2.5f * MM_PER_INCH;
    public static final float CAMERA_LEFT_DISPLACEMENT = 3.0f * MM_PER_INCH;

    // Constants for perimeter targets
    public static final float MM_TARGET_HEIGHT = 6 * MM_PER_INCH;
    public static final float HALF_FIELD = 72 * MM_PER_INCH;
    public static final float HALF_TILE = 12 * MM_PER_INCH;
    public static final float ONE_AND_HALF_TILE = 36 * MM_PER_INCH;

}
