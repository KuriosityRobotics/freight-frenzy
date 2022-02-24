package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.util.function.Function;
import java.util.function.Supplier;

public interface PoseProvider {
    Pose getPose();
    Pose getVelocity();

    default Rotation getRotation() {
        return new Rotation(new Vector3D(0, -1, 0), getPose().heading, RotationConvention.VECTOR_OPERATOR);
    }

    default Vector3D getTranslation() {
        return new Vector3D(getPose().x, 0, getPose().y);
    }

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
