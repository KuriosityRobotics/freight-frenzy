package com.kuriosityrobotics.firstforward.robot.debug.telemetry;

import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

public interface PoseWatcher {
    void sendPose(Pose pose);
}
