package com.kuriosityrobotics.firstforward.robot.util.motionprofiling;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;

class MotionPoint extends Point {
    AngleLock angleLock;
    double velocity;

    public MotionPoint(Point point, double velocity, AngleLock angleLock) {
        super(point.x, point.y);

        this.velocity = velocity;

        this.angleLock = angleLock;
    }
}