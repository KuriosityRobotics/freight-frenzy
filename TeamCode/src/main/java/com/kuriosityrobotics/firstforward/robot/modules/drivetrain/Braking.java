package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import com.kuriosityrobotics.firstforward.robot.util.PID.ClassicalPID;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

final class Braking {
    private ClassicalPID angularBrakeController;
    private ClassicalPID distanceBrakeController;

    private Pose brakePose;

    boolean isBraking() {
        return brakePose != null;
    }

    void stopBraking() {
        brakePose = null;
        distanceBrakeController = null;
        angularBrakeController = null;
    }

    Pose getBrakeMovement(boolean isDriverControlled, Pose currentPose, Pose velocity) {
        // we stop braking when the velocity is low (brake is a synonym for 'stop')
        if (Math.hypot(velocity.x, velocity.y) < 1 && velocity.heading < Math.PI / 12) {
            if (isBraking())
                stopBraking();

            return Pose.ZERO;
        }

        // we want to save the pose we start braking at (the pose the robot is in when mvoements are 0)
        // at this point we know we want to brake
        if (!isBraking()) {
            this.brakePose = currentPose.add(velocity.scale(.1)); // we add .1 seconds worth of movement to make it feel snappy
            angularBrakeController = new ClassicalPID(0.019, 0, 0.1);
            distanceBrakeController = new ClassicalPID(0.015, 0, 0.7);

            if (isDriverControlled)
                this.brakePose = currentPose.add(velocity.scale(.1)); // we add .1 seconds worth of movement to make it feel snappy
        }

        double moveSpeed = distanceBrakeController.calculateSpeed(currentPose.distance(brakePose)); // to use for PID
        Point translationSpeed = currentPose.relativeComponentsToPoint(brakePose).scale(moveSpeed);

        double rotationSpeed = angularBrakeController.calculateSpeed(angleWrap(brakePose.heading - currentPose.heading));

        return new Pose(translationSpeed, rotationSpeed);
    }

    Pose getBrakePose() {
        return brakePose;
    }
}
