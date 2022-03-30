package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain implements Module, Telemeter {
    private final LocationProvider locationProvider;
    private final DrivetrainModule drivetrainModule;

    //states
    private ConstrainedMovementCalculator.WheelMovements wheelMovements;

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
        wheelMovements = ConstrainedMovementCalculator.WheelMovements.fromMovements(xMov, yMov, turnMov);
    }

    public void setMovements(ConstrainedMovementCalculator.WheelMovements movements) {
        wheelMovements = movements;
    }

    @Override
    public void onStart() {
        this.opmodeStarted = true;
    }

    private boolean movementsZero() {
        return doublesEqual(getxMov(), 0) &&
                doublesEqual(getyMov(), 0) &&
                doublesEqual(getTurnMov(), 0);
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
                drivetrainModule.setMovements(wheelMovements);
            }

            stallDetector.update(locationProvider.getVelocity(), getxMov(), getyMov(), getTurnMov());
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

        data.add(String.format("xMov: %s, yMov: %s, turnMov: %s", getxMov(), getyMov(), getTurnMov()));

        data.add("--");

        data.add("Braking: " + brake.isBraking());
        data.add("Brake Pose: " + brake.getBrakePose());

        data.add("--");
        data.add("Stall Status: " + stallDetector.isStalled());

        return data;
    }

    @Override
    public int getShowIndex() {
        return 1;
    }

    public double getxMov() {
        return wheelMovements.xMov();
    }

    public void setxMov(double xMov) {
        setMovements(xMov, getyMov(), getTurnMov());
    }

    public double getyMov() {
        return wheelMovements.yMov();
    }

    public void setyMov(double yMov) {
        setMovements(getxMov(), yMov, getTurnMov());
    }

    public double getTurnMov() {
        return wheelMovements.angularMov();
    }

    public void setTurnMov(double turnMov) {
        this.setMovements(getxMov(), getyMov(), turnMov);
    }
}
