package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain implements Module, Telemeter {
    Robot robot;
    public final DrivetrainModule drivetrainModule;

    //states
    public volatile double xMov;
    public volatile double yMov;
    public volatile double turnMov;

    //braking states
    public Pose desiredBrakePose;

    //saves
    private double timeStopped = 0;

    //braking controller
    ClassicalPID angularBrakeController = new ClassicalPID(0.5, 0, 1);
    ClassicalPID distanceBrakeController = new ClassicalPID(0.03, 0, 0.7);

    public Drivetrain(Robot robot, Pose desiredBrakePose) {
        this.robot = robot;
        drivetrainModule = new DrivetrainModule(robot);
        robot.telemetryDump.registerTelemeter(this);
        this.desiredBrakePose = desiredBrakePose;
    }

    public void setMovements(double xMov, double yMov, double turnMov) {
        this.xMov = xMov;
        this.yMov = yMov;
        this.turnMov = turnMov;
    }

    private boolean brakingRequested() {
        //return xMov == 0
        //        && yMov == 0
        //        && turnMov == 0;
        return false;
    }

    // updates drivetrainModule and odometry
    // gets updated in robot
    public void update() {
        if (drivetrainModule.isOn()) {
            if (brakingRequested()) {
                setMovementTowardsBrake();
            } else {
                drivetrainModule.setMovements(xMov, yMov, turnMov);
                //reset PIDs since its a new point
                distanceBrakeController.reset();
                angularBrakeController.reset();
                desiredBrakePose = getCurrentPose(); // brakePose holds the pose we were at last time we were not braking
            }
            drivetrainModule.update();
        }
    }

    // uses pid to go to point
    // used for braking
    private void setMovementTowardsBrake() {
        Pose currentPosition = getCurrentPose();

        double moveSpeed = distanceBrakeController.calculateSpeed(currentPosition.distance(desiredBrakePose)) * 0.55; // to use for PID
        double turnSpeed = angularBrakeController.calculateSpeed(desiredBrakePose.heading - currentPosition.heading) * 0.65;

        Point components = relativeComponentsToPoint(desiredBrakePose).scale(moveSpeed);
        double xError = components.x;
        double yError = components.y;

        if(xError < .2)
            xError = 0;
        if(yError < .2)
            yError = 0;

        drivetrainModule.setMovements(xError, yError, turnSpeed);
    }

    /**
     * Get the global absolute angle of the line between the robot's position and the given point,
     * where 0 is along the y axis.
     *
     * @return absolute heading to that point
     */
    public double absoluteHeadingToPoint(Point point) {
        Point currentPosition = getCurrentPose();

        return Math.atan2(point.x - currentPosition.x, point.y - currentPosition.y);
    }

    /**
     * The heading the robot would have to turn by to face the point directly.
     *
     * @return relative heading to that point
     */
    public double relativeHeadingToPoint(Point point) {
        Pose currentPosition = getCurrentPose();

        double absoluteHeadingToPoint = absoluteHeadingToPoint(point);

        return angleWrap(absoluteHeadingToPoint - currentPosition.heading);
    }

    public Point relativeComponentsToPoint(Point point) {
        Pose currentPosition = getCurrentPose();

        double relativeHeadingToPoint = relativeHeadingToPoint(point);

        double distanceError = currentPosition.distance(point);

        double xError = distanceError * Math.sin(relativeHeadingToPoint);
        double yError = distanceError * Math.cos(relativeHeadingToPoint);

        return new Point(xError, yError);
    }

    public double distanceToPoint(Point point) {
        return this.getCurrentPose().distance(point);
    }

    public Pose getCurrentPose() {
        return robot.sensorThread.getPose();
    }

    public Pose getVelocity() {
        return robot.sensorThread.getVelocity();
    }

    public double getOrthVelocity() {
        Pose velo = getVelocity();
        return Math.sqrt(Math.pow(velo.x, 2) + Math.pow(velo.y, 2));
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }

    @Override
    public List<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add(String.format("xMov: %s, yMov: %s, turnMov: %s", xMov, yMov, turnMov));

        data.add("--");

        data.add("Braking: " + brakingRequested());
        data.add("Brake Pose: " + desiredBrakePose);

        return data;
    }
}