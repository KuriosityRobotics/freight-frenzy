package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;

import java.util.ArrayList;

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

    //braking controller
    //not tuned yet
    //ClassicalPID angularBrakeController = new ClassicalPID(0, 0, 0);
    //ClassicalPID distanceBrakeController = new ClassicalPID(0, 0, 0);

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
        Point components = relativeComponentsToPoint(point);

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

    //updates drivetrainModule and odometry
    //gets updated in robot
    public void update() {
        if (xMov == 0 && yMov == 0 && turnMov == 0 && zeroPowerBrake && opmodeStarted) {
            if (!brake) {
                this.brake = true;
                brakePose = getCurrentPose();
            }
        } else {
            this.brake = false;
        }

        if (drivetrainModule.isOn()) {
            if (brake) {
                setMovementTowardsBrake();
            } else {
                drivetrainModule.setMovements(xMov, yMov, turnMov);
                //reset PIDs since its a new point
                //distanceBrakeController.reset();
                //angularBrakeController.reset();
            }

            drivetrainModule.update();
        }
    }

    //uses pid to go to point
    //used for braking
    private void setMovementTowardsBrake() {
        Pose currentPosition = getCurrentPose();

        double distanceError = currentPosition.distance(brakePose); // to use for PID
        double headingError = brakePose.heading - currentPosition.heading;

        Point components = relativeComponentsToPoint(brakePose);
        double xError = components.x;
        double yError = components.y;

        double total = Math.abs(xError) + Math.abs(yError);

        double xPow = (xError / total);
        double yPow = (yError / total);
        double turnPow = headingError;

        drivetrainModule.setMovements(xPow, yPow, turnPow);
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

    public Point relativeComponentsToPoint(Point point) {
        Pose currentPosition = getCurrentPose();

        double absoluteHeadingToPoint = absoluteHeadingToPoint(point);
        double relativeHeadingToPoint = absoluteHeadingToPoint - currentPosition.heading;

        double distanceError = currentPosition.distance(point);

        double xError = distanceError * Math.sin(relativeHeadingToPoint);
        double yError = distanceError * Math.cos(relativeHeadingToPoint);

        return new Point(xError, yError);
    }

    //calculate angle from target (center) to pos
    //used in goToPoint() and moveTowardsPoint()
    // TODO remove?????
    private double absRadDifference(Point pos, Point target) {
        double rad;
        double add = 0;
        if (Math.abs(target.y - pos.y) < .00001) {
            rad = 0;
        } else {
            rad = Math.atan((target.y - pos.y) / (target.x - pos.x));
        }
        if (target.x < pos.x) {
            add = Math.PI;
        }
        return angleWrap(rad + add);
    }

    public Pose getCurrentPose() {
        return robot.getSensorThread().odometry.getPose();
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
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("xMov: " + xMov);
        data.add("yMov: " + yMov);
        data.add("turnMov: " + turnMov);

        data.add("--");
        data.add("Braking: " + brake);
        data.add("Brake Pose: " + brakePose);

        return data;
    }

    //    public class Movements {
//        double xMovement, yMovement, turnMovement;
//
//        public Movements(double xMovement, double yMovement, double turnMovement) {
//            this.xMovement = xMovement;
//            this.yMovement = yMovement;
//            this.turnMovement = turnMovement;
//        }
//    }
}