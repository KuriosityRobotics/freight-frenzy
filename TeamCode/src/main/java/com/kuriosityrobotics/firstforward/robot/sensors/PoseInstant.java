package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

class PoseInstant extends Pose {
    double timestamp;

    public PoseInstant(Pose pose, double timestamp) {
        super(pose);

        this.timestamp = timestamp;
    }
}
