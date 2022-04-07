package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile.ROBOT_MAX_ACCEL;
import static com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile.ROBOT_MAX_DECCEL;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.rotate;
import static java.lang.Math.pow;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class Odometry extends RollingVelocityCalculator implements Module, LocationProvider {
    // Constants
    private static final double INCHES_PER_ENCODER_TICK = 0.0007284406721 * (100.0 / 101.9889);
    private static final double LR_ENCODER_DIST_FROM_CENTER = 2.4367390324945863115640954444386315979381132912686705686229716745 * (3631.6415304167253 / 3600.);
    private static final double B_ENCODER_DIST_FROM_CENTER = 2.9761730787305137386664648856740921319601002407647215925090672904 * (3622.009011720834 / 3600.);
    // Encoders
    private final DcMotor yLeftEncoder;
    private final DcMotor yRightEncoder;
    private final DcMotor mecanumBackEncoder;
    private final DcMotor mecanumFrontEncoder;
    private final ExtendedKalmanFilter kalmanFilter;
    private final IMU imu;
    // Position of the robot, can be changed through constructor
    private double worldX;
    private double worldY;
    private double worldHeadingRad;
    // veloclity of the robot
    private double xVel = 0;
    private double yVel = 0;
    private double angleVel = 0;
    // old vel for acceleration calculations
    private double oldxVel = 0;
    private double oldyVel = 0;
    private double oldangleVel = 0;
    // change in position of the robot
    private double dx = 0;
    private double dy = 0;
    private double dHeading = 0;
    // For position calculation
    private double lastLeftPosition = 0;
    private double lastRightPosition = 0;
    private double lastMecanumBackPosition = 0;
    private double lastMecanumFrontPosition = 0;
    // For velocity calculation
    private double oldX = 0;
    private double oldY = 0;
    private double oldHeading = 0;

    //max vel, accel, deccel
    private double maxVel = 0;
    private double maxAccel = 0;
    private double minAccel = 0; //deccel

    private double lastUpdateTime = 0;
    private double thetaLR = 0;
    private double thetaFB = 0;

    public Odometry(HardwareMap hardwareMap, Pose pose, ExtendedKalmanFilter kalmanFilter, IMU imu) {
        this.worldX = pose.x;
        this.worldY = pose.y;
        this.worldHeadingRad = pose.heading;
        this.imu = imu;

        yLeftEncoder = hardwareMap.get(DcMotor.class, "fLeft");
        yRightEncoder = hardwareMap.get(DcMotor.class, "fRight");
        mecanumBackEncoder = hardwareMap.get(DcMotor.class, "bLeft");
        mecanumFrontEncoder = hardwareMap.get(DcMotor.class, "bRight");
        this.kalmanFilter = kalmanFilter;

        resetEncoders();

        FileDump.addField("xVel", this);
        FileDump.addField("yVel", this);
    }

    private double accelerationSigmoid(double dq, double accel) {
        if (accel > 0)
            return dq * (imu.isOffGround() /*|| accel <= 2 * ROBOT_MAX_ACCEL*/ ? 2 : .8);
        else
            return dq * (imu.isOffGround() /*|| -accel <= 2 * ROBOT_MAX_DECCEL*/ ? .8 : 2);

    }

    public void update() {
        var now = SystemClock.elapsedRealtime();
        var dt  = (now - lastUpdateTime) / 1000.;
        if (lastUpdateTime == 0)
            dt = 1. / maxFrequency();

        calculatePosition();

        var accel = imu.getAcceleration();
        var headingDeviation = accelerationSigmoid(dHeading, Math.hypot(accel.x, accel.y));

        kalmanFilter.builder()
                .time(now)
                .mean(dx, dy, dHeading)
                .outputToState(rotate(kalmanFilter.outputVector()[2]))
                .variance(
                        pow(dt * accel.x * .3, 2),
                        pow(dt * accel.y * .3, 2),
                        pow(headingDeviation, 2)
                )
                .predict();

        calculateInstantaneousVelAccelDeccel();
        this.calculateRollingVelocity(new PoseInstant(getPose(), SystemClock.elapsedRealtime() / 1000.0));
    }

    private void calculatePosition() {
        // if odometry output is wrong, no worries, just find out which one needs to be reversed
        double newLeftPosition = yLeftEncoder.getCurrentPosition();
        double newRightPosition = yRightEncoder.getCurrentPosition();
        double newMecanumBackPosition = mecanumBackEncoder.getCurrentPosition();
        double newMecanumFrontPosition = mecanumFrontEncoder.getCurrentPosition();

        double deltaLeftPosition = newLeftPosition - lastLeftPosition;
        double deltaRightPosition = newRightPosition - lastRightPosition;
        double deltaMecanumBackPosition = newMecanumBackPosition - lastMecanumBackPosition;
        double deltaMecanumFrontPosition = newMecanumFrontPosition - lastMecanumFrontPosition;

        updateWorldPosition(deltaLeftPosition, deltaRightPosition, deltaMecanumBackPosition, deltaMecanumFrontPosition);

        lastLeftPosition = newLeftPosition;
        lastRightPosition = newRightPosition;
        lastMecanumBackPosition = newMecanumBackPosition;
        lastMecanumFrontPosition = newMecanumFrontPosition;
    }

    private void calculateInstantaneousVelAccelDeccel() {
        long currentUpdateTime = SystemClock.elapsedRealtime();
        double dTime = (currentUpdateTime - lastUpdateTime) / 1000.;

        // normalize to milliseconds then apply dimensional analysis to get to seconds
        xVel = (worldX - oldX) / (dTime);
        yVel = (worldY - oldY) / (dTime);
        angleVel = (worldHeadingRad - oldHeading) / (dTime);

        oldX = worldX;
        oldY = worldY;
        oldHeading = worldHeadingRad;

        maxVel = Math.max(maxVel, getVelMag());
        double accel = (getVelMag() - Math.hypot(oldxVel, oldyVel)) / (dTime);
        if (accel > 0){
            maxAccel = Math.max(accel, maxAccel);
        }else{
            minAccel = Math.min(accel, minAccel);
        }

        oldxVel = xVel;
        oldyVel = yVel;
        oldangleVel = angleVel;

        lastUpdateTime = currentUpdateTime;
    }

    private void updateWorldPosition(double dLeftPod, double dRightPod, double dMecanumBackPod, double dMecanumFrontPod) {
        // convert all inputs to inches
        double dLeftPodInches = dLeftPod * INCHES_PER_ENCODER_TICK;
        double dRightPodInches = dRightPod * INCHES_PER_ENCODER_TICK;
        double dMecanumBackPodInches = dMecanumBackPod * INCHES_PER_ENCODER_TICK;
        double dMecanumFrontPodInches = dMecanumFrontPod * INCHES_PER_ENCODER_TICK;

        // so its easier to type
        double P = LR_ENCODER_DIST_FROM_CENTER;
        double Q = B_ENCODER_DIST_FROM_CENTER;

        // find robot relative deltas
        double dThetaLR = (dLeftPodInches - dRightPodInches) / (2 * P);
        double dThetaFB = (dMecanumFrontPodInches - dMecanumBackPodInches) / (2 * Q);

        thetaLR += dThetaLR;
        thetaFB += dThetaFB;

        double X = Math.abs(dLeftPodInches + dRightPodInches);
        double Y = Math.abs(dMecanumFrontPodInches + dMecanumBackPodInches);

        double weightLR = 0.5;
        double weightFB = 0.5;

        if (X + Y != 0) {
            weightLR = X / (X + Y);
            weightFB = Y / (X + Y);
        }

        double dTheta = weightLR * dThetaLR + weightFB * dThetaFB;

        double dRobotX = dMecanumBackPodInches * sinXOverX(dTheta) + Q * Math.sin(dTheta) - dLeftPodInches * cosXMinusOneOverX(dTheta) + P * (Math.cos(dTheta) - 1);
        double dRobotY = dLeftPodInches * sinXOverX(dTheta) - P * Math.sin(dTheta) + dMecanumBackPodInches * cosXMinusOneOverX(dTheta) + Q * (Math.cos(dTheta) - 1);

        // change global variables so they can be used in the kalman filter
        dx = dRobotX;
        dy = dRobotY;
        dHeading = dTheta;

        worldX += dRobotX * Math.cos(worldHeadingRad) + dRobotY * Math.sin(worldHeadingRad);
        worldY += dRobotY * Math.cos(worldHeadingRad) - dRobotX * Math.sin(worldHeadingRad);
        worldHeadingRad = worldHeadingRad + dTheta;
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double sinXOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = 1;
            double bottom = 1;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 2) * (2 * i + 3);
            }
            return retVal;
        } else {
            return Math.sin(x) / x;
        }
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double cosXMinusOneOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = -x;
            double bottom = 2;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 3) * (2 * i + 4);
            }
            return retVal;
        } else {
            return (Math.cos(x) - 1) / x;
        }
    }

    private void resetEncoders() {
        yLeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yRightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumBackEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumBackEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumFrontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeftPosition = 0;
        lastRightPosition = 0;
        lastMecanumBackPosition = 0;
        lastMecanumFrontPosition = 0;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add(getPose().toString("odometry pose"));

        data.add("max vel: " + maxVel);
        data.add("max accel: " + maxAccel);
        data.add("max deccel: " + -minAccel);

        return data;
    }

    /**
     * Get the robot's current pose, containing x, y, and heading.
     * <p>
     * There is only one getter method to reduce the likelihood of values changing between getters.
     */
    public Pose getPose() {
        return new Pose(worldX, worldY, worldHeadingRad);
    }

    public void setPose(Pose pose) {
        this.oldX = pose.x;
        this.oldY = pose.y;
        this.oldHeading = pose.heading;

        this.worldX = pose.x;
        this.worldY = pose.y;
        this.worldHeadingRad = pose.heading;
    }

    public Pose getVelocity() {
        return getInstantaneousVelocity();
    }

    public Pose getInstantaneousVelocity() {
        return new Pose(xVel, yVel, angleVel);
    }

    public double getVelMag() {
        return Math.hypot(xVel, yVel);
    }

    @Override
    public String getName() {
        return "Odometry";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}