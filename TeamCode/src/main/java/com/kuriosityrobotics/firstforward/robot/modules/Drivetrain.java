package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.mean;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.sd;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;

import org.apache.commons.collections4.queue.CircularFifoQueue;

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

    // stall detector
    private static final int STALL_DETECTOR_CAPACITY = 300;
    private static final double STALL_EPSION = 0.1;
    private final CircularFifoQueue<Double> poseXSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> poseYSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> poseHeadingSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);

    public Drivetrain(Robot robot, Pose brakePose) {
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

            drivetrainModule.update();

            poseXSD.add(getCurrentPose().x);
            poseYSD.add(getCurrentPose().y);
            poseHeadingSD.add(getCurrentPose().heading);
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

    public boolean isStalled() {
        boolean movementsNotSet = doublesEqual(xMov, 0) && doublesEqual(yMov, 0) && doublesEqual(turnMov, 0);

        boolean isXStalled = mean(poseXSD) < STALL_EPSION;
        boolean isYStalled = mean(poseYSD) < STALL_EPSION;
        boolean isHeadingStalled = mean(poseHeadingSD) < STALL_EPSION;
        boolean isStalled = isXStalled && isYStalled && isHeadingStalled;

        return !movementsNotSet && isStalled;
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
        data.add("Stall Status: " + isStalled());

        return data;
    }
}
