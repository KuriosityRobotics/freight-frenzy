package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

public interface PoseWatcher {
    void sendPose(Pose pose);
}
