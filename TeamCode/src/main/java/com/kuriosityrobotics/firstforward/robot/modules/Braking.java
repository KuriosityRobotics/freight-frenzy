package com.kuriosityrobotics.firstforward.robot.modules;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;

public final class Braking {
    private ClassicalPID angularBrakeController;
    private ClassicalPID distanceBrakeController;

    private Pose brakePose;

    boolean isBraking() {
        return brakePose != null;
    }

    private void stopBraking() {
        brakePose = null;
        distanceBrakeController = null;
        angularBrakeController = null;
    }

    Pose getBrakeMovement(Pose currentPose, Pose velocity) {
        // we stop braking when the velocity is low (brake is a synonym for 'stop')
        // TODO:  tune this constant
        if (Math.hypot(velocity.x, velocity.y) < 0.1) {
            if (isBraking())
                stopBraking();

            return Pose.ZERO;
        }

        // we want to save the pose we start braking at (the pose the robot is in when mvoements are 0)
        // at this point we know we want to brake
        if (!isBraking()) {
            this.brakePose = currentPose;
            angularBrakeController = new ClassicalPID(0.5, 0, 1);
            distanceBrakeController = new ClassicalPID(0.03, 0, 0.7);
        }

        double moveSpeed = distanceBrakeController.calculateSpeed(currentPose.distance(brakePose)) * 0.55; // to use for PID
        Point translationSpeed = currentPose.relativeComponentsToPoint(brakePose).scale(moveSpeed);

        double rotationSpeed = angularBrakeController.calculateSpeed(brakePose.heading - currentPose.heading) * 0.65;

        return new Pose(translationSpeed, rotationSpeed);
    }

    Pose getBrakePose() {
        return brakePose;
    }
}
