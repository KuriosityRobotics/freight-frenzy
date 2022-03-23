package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.geometry.euclidean.threed.rotation.Rotation3D;

import java.util.function.Supplier;

@SuppressWarnings("unused")
public interface LocationProvider {
    Pose getPose();
    Pose getVelocity();

    default double distanceToPoint(Point point) {
        return getPose().distance(point);
    }

    default double getOrthVelocity() {
        Pose velo = getVelocity();
        return Math.sqrt(Math.pow(velo.x, 2) + Math.pow(velo.y, 2));
    }

    default Rotation3D getRotation() {
        return QuaternionRotation.fromAxisAngle(Vector3D.of(0, -1, 0), getPose().heading);
    }

    default Vector3D getTranslation() {
        return Vector3D.of(getPose().x, 0, getPose().y);
    }

    static LocationProvider of(Supplier<Pose> pose, Supplier<Pose> velocity) {
        return new LocationProvider() {
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

    static LocationProvider of(Pose pose, Pose velocity) {
        return new LocationProvider() {
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
