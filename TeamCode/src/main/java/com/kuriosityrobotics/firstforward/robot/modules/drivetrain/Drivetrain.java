package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import static org.apache.commons.math3.analysis.FunctionUtils.add;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Drivetrain implements Module, Telemeter {
    final LocationProvider locationProvider;
    private final DrivetrainModule drivetrainModule;

    //states
    public volatile double xMov, yMov, turnMov;

    //braking states
    private final Braking brake = new Braking(); // whether or not to actively brake
    private boolean opmodeStarted = false;

    // stalling states
    private final StallDetector stallDetector = new StallDetector();

    public Drivetrain(LocationProvider locationProvider, HardwareMap hardwareMap) {
        this.locationProvider = locationProvider;
        drivetrainModule = new DrivetrainModule(hardwareMap);
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
            if (movementsZero() && !locationProvider.getVelocity().equals(Pose.ZERO)) {
                Pose brakeMovements = brake.getBrakeMovement(locationProvider.getPose().wrapped(), locationProvider.getVelocity());
                drivetrainModule.setMovements(brakeMovements);
            } else {
                if (brake.isBraking())
                    brake.stopBraking();
                drivetrainModule.setMovements(xMov, yMov, turnMov);
            }

            //stallDetector.update(getCurrentPose(), xMov, yMov, turnMov);
            drivetrainModule.update();
        }
    }

    public StallDetector getStallDetector() {
        return this.stallDetector;
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

        data.add("Braking: " + brake.isBraking());
        data.add("Brake Pose: " + brake.getBrakePose());

        data.add("--");
        data.add("Stall Status: " + stallDetector.isStalled());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        return new HashMap<>() {{
            put("m1", drivetrainModule.m1);
            put("m2", drivetrainModule.m2);
            put("m3", drivetrainModule.m3);
            put("m4", drivetrainModule.m4);

        }};
    }
}
