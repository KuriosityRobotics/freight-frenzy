package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain implements Module, Telemeter {
    Robot robot;
    private final boolean isOn = true;
    public final DrivetrainModule drivetrainModule;

    //states
    public double xMov;
    public double yMov;
    public double turnMov = 0;
    public boolean zeroPowerBrake = true;

    //braking states
    private boolean brake = false; // whether or not to actively brake
    public Pose brakePose;
    private boolean opmodeStarted = false;

    //saves
    private double timeStopped = 0;

    //braking controller
    ClassicalPID angularBrakeController = new ClassicalPID(0.5, 0, 1);
    ClassicalPID distanceBrakeController = new ClassicalPID(0.03, 0, 0.7);

    public Drivetrain(Robot robot) {
        this.robot = robot;

        drivetrainModule = new DrivetrainModule(robot);

        robot.telemetryDump.registerTelemeter(this);
    }

    public void setMovements(double xMov, double yMov, double turnMov) {
        this.xMov = xMov;
        this.yMov = yMov;
        this.turnMov = turnMov;
    }

    /**
     * Sets x and y states to move towards a target point.
     *
     * @param point
     * @param moveSpeed
     */
    public void setMovementsTowardPoint(Point point, double moveSpeed) {
        Point components = relativeComponentsToPoint(point).scale(moveSpeed);

        double x = components.x;
        double y = components.y;

        double total = Math.abs(x) + Math.abs(y);

        double xPow = (x / total) * moveSpeed;
        double yPow = (y / total) * moveSpeed;

        this.xMov = xPow;
        this.yMov = yPow;
    }

    /**
     * Sets turn state to a desired heading.
     *
     * @param heading
     */
    public void setMovementTowardHeading(double heading) {
        Pose current = getCurrentPose();

        double headingError = heading - current.heading;

        this.turnMov = headingError; // TODO multiply by factor?
    }

    @Override
    public void onStart() {
        this.opmodeStarted = true;
    }

    // updates drivetrainModule and odometry
    // gets updated in robot
    public void update() {
        if (xMov == 0 && yMov == 0 && turnMov == 0 && zeroPowerBrake && opmodeStarted) {
            if (!brake) {
                this.brake = true;
            } else {
                //stops robot when it's at low vel
                if (distanceBrakeController.getD() > 0) {
                    this.brake = false;
                }
            }
        } else {
            this.brake = false;
        }

//        Log.v("Drivetrain", "Iteration called");
        if (drivetrainModule.isOn()) {
            if (brake) {
                setMovementTowardsBrake();
            } else {
                drivetrainModule.setMovements(xMov, yMov, turnMov);
                //reset PIDs since its a new point
                distanceBrakeController.reset();
                angularBrakeController.reset();
                brakePose = getCurrentPose();
            }
            drivetrainModule.update();
        }
    }

    // uses pid to go to point
    // used for braking
    private void setMovementTowardsBrake() {
        Pose currentPosition = getCurrentPose();
        if (brakePose == null) {
            brakePose = currentPosition;
            return;
        }

        double moveSpeed = distanceBrakeController.calculateSpeed(currentPosition.distance(brakePose)) * 0.55; // to use for PID
        double turnSpeed = angularBrakeController.calculateSpeed(brakePose.heading - currentPosition.heading) * 0.65;

        Point components = relativeComponentsToPoint(brakePose).scale(moveSpeed);
        double xError = components.x;
        double yError = components.y;

        drivetrainModule.setMovements(xError, yError, turnSpeed);
    }

    public void setBrakePose(Pose brakePose) {
        this.brakePose = brakePose;
    }

    /**
     * Get the global absolute angle of the line between the robot's position and the given point,
     * where 0 is along the y axis.
     *
     * @param point
     * @return absolute heading to that point
     */
    public double absoluteHeadingToPoint(Point point) {
        Point currentPosition = getCurrentPose();

        return Math.atan2(point.x - currentPosition.x, point.y - currentPosition.y);
    }

    /**
     * The heading the robot would have to turn by to face the point directly.
     *
     * @param point
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
        return isOn;
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

        data.add("Braking: " + brake);
        data.add("Brake Pose: " + brakePose);

        return data;
    }
}