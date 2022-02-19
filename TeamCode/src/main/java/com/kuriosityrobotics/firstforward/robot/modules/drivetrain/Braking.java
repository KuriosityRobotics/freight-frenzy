package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;

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

    Pose getBrakeMovement(Pose currentPose, Pose velocity) {
        return new Pose(0, 0, 0);
        /*
        // we stop braking when the velocity is low (brake is a synonym for 'stop')
        // TODO:  tune this constant
        if (Math.hypot(velocity.x, velocity.y) < 0.1 && velocity.heading < Math.PI / 12) {
            if (isBraking())
                stopBraking();

            return Pose.ZERO;
        }

        // we want to save the pose we start braking at (the pose the robot is in when mvoements are 0)
        // at this point we know we want to brake
        if (!isBraking()) {
            this.brakePose = currentPose.add(velocity.scale(.1)); // we add .1 seconds worth of movement to make it feel snappy
            angularBrakeController = new ClassicalPID(0.5, 0, 1);
            distanceBrakeController = new ClassicalPID(0.02, 0, 0.7);
        }

        double moveSpeed = distanceBrakeController.calculateSpeed(currentPose.distance(brakePose)) * 0.55; // to use for PID
        Point translationSpeed = currentPose.relativeComponentsToPoint(brakePose).scale(moveSpeed);

        double rotationSpeed = angularBrakeController.calculateSpeed(angleWrap(brakePose.heading - currentPose.heading)) * 0.65;

        return new Pose(translationSpeed, rotationSpeed);

         */
    }

    Pose getBrakePose() {
        return brakePose;
    }
}
