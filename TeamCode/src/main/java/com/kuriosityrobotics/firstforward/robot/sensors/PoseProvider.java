package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

import java.util.function.Function;
import java.util.function.Supplier;

public interface PoseProvider {
    Pose getPose();
    Pose getVelocity();

    static PoseProvider of(Supplier<Pose> pose, Supplier<Pose> velocity) {
        return new PoseProvider() {
            @Override
            public Pose getPose() {
                return pose.get();
            }

            @Override
            public Pose getVelocity() {
                return velocity.get();
            }
        };
    }

    static PoseProvider of(Pose pose, Pose velocity) {
        return new PoseProvider() {
            @Override
            public Pose getPose() {
                return pose;
            }

            @Override
            public Pose getVelocity() {
                return velocity;
            }
        };
    }

}
