package com.kuriosityrobotics.firstforward.robot;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.util.function.Supplier;

public interface LocationProvider {
    public abstract Pose getPose();
    public abstract Pose getVelocity();

    public default double distanceToPoint(Point point) {
        return getPose().distance(point);
    }

    public default double getOrthVelocity() {
        Pose velo = getVelocity();
        return Math.sqrt(Math.pow(velo.x, 2) + Math.pow(velo.y, 2));
    }

    public default Rotation getRotation() {
        return new Rotation(new Vector3D(0, -1, 0), getPose().heading, RotationConvention.VECTOR_OPERATOR);
    }

    public default Vector3D getTranslation() {
        return new Vector3D(getPose().x, 0, getPose().y);
    }

    public static LocationProvider of(Supplier<Pose> pose, Supplier<Pose> velocity) {
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

    public static LocationProvider of(Pose pose, Pose velocity) {
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
