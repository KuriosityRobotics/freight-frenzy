package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.mean;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;

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

    //braking states
    private Braking brake = new Braking(); // whether or not to actively brake
    private boolean opmodeStarted = false;

    // stalling states
    private StallDetector stallDetector = new StallDetector();

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

    @Override
    public void onStart() {
        this.opmodeStarted = true;
    }

    private boolean movementsZero() {
        return doublesEqual(xMov, 0) &&
                doublesEqual(yMov, 0) &&
                doublesEqual(turnMov, 0);
    }

    // updates drivetrainModule and odometry
    // gets updated in robot
    public void update() {
        if (opmodeStarted) {
            if (movementsZero() && !getVelocity().equals(Pose.ZERO)) {
//                Pose brakeMovements = brake.getBrakeMovement(getCurrentPose().wrapped(), getVelocity());
//                drivetrainModule.setMovements(brakeMovements);
                drivetrainModule.setMovements(0, 0, 0);
            } else {
                if (brake.isBraking())
                    brake.stopBraking();
                drivetrainModule.setMovements(xMov, yMov, turnMov);
            }

            stallDetector.update(getCurrentPose(), xMov, yMov, turnMov);
            drivetrainModule.update();
        }
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

        data.add("Braking: " + brake.isBraking());
        data.add("Brake Pose: " + brake.getBrakePose());

        data.add("--");
        data.add("Stall Status: " + stallDetector.isStalled());

        return data;
    }
}
