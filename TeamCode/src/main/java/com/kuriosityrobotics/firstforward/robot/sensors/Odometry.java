package com.kuriosityrobotics.firstforward.robot.sensors;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

public class Odometry implements Telemeter {
    // Encoders
    private final DcMotor yLeftEncoder;
    private final DcMotor yRightEncoder;
    private final DcMotor mecanumBackEncoder;
    private final DcMotor mecanumFrontEncoder;

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

    // acceleration calculations
    private double xAccel = 0;
    private double yAccel = 0;
    private double angleAccel = 0;

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

    private double lastUpdateTime = 0;

    // Constants
    private static final double INCHES_PER_ENCODER_TICK = 0.0007284406721 * (100.0 / 101.9889);
    private static final double LR_ENCODER_DIST_FROM_CENTER = (4.75 / 2) * (740. / 720.) * (363. / 360) * (360. / 358.) * (356. / 360);
    private static final double B_ENCODER_DIST_FROM_CENTER = 3 * (1786.59 / 1800.);

    public Odometry(HardwareMap hardwareMap, Pose pose) {
        this.worldX = pose.x;
        this.worldY = pose.y;
        this.worldHeadingRad = pose.heading;

        yLeftEncoder = hardwareMap.get(DcMotor.class, "fLeft");
        yRightEncoder = hardwareMap.get(DcMotor.class, "fRight");
        mecanumBackEncoder = hardwareMap.get(DcMotor.class, "bLeft");
        mecanumFrontEncoder = hardwareMap.get(DcMotor.class, "bRight");

        resetEncoders();

        FileDump.addField("xVel", this);
        FileDump.addField("yVel", this);
        FileDump.addField("angleVel", this);
        FileDump.addField("xAccel", this);
        FileDump.addField("yAccel", this);
        FileDump.addField("angleAccel", this);
    }

    public void update() {
        calculatePosition();

        calculateInstantaneousVelocity();

//        this.calculateRollingVelocity(new PoseInstant(getPose(), SystemClock.elapsedRealtime() / 1000.0));
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

    private void calculateInstantaneousVelocity() {
        long currentUpdateTime = SystemClock.elapsedRealtime();
        double dTime = (currentUpdateTime - lastUpdateTime) / 1000;

        // normalize to milliseconds then apply dimensional analysis to get to seconds
        xVel = (worldX - oldX) / (dTime);
        yVel = (worldY - oldY) / (dTime);
        angleVel = (worldHeadingRad - oldHeading) / (dTime);

        xAccel = (xVel - oldxVel) / (dTime);
        yAccel = (yVel - oldyVel) / (dTime);
        angleAccel = (angleVel - oldangleVel) / (dTime);

        oldX = worldX;
        oldY = worldY;
        oldHeading = worldHeadingRad;

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
        double L = dLeftPodInches;
        double R = dRightPodInches;
        double B = dMecanumBackPodInches;
        double F = dMecanumFrontPodInches;
        double P = LR_ENCODER_DIST_FROM_CENTER;
        double Q = B_ENCODER_DIST_FROM_CENTER;

        // find robot relative deltas
        double dThetaLR = (L - R) / (2 * P);
        double dThetaFB = (F - B) / (2 * Q);

        double X = Math.abs(L + R);
        double Y = Math.abs(F + B);

        double weightLR = 0.5;
        double weightFB = 0.5;

        if (X+Y != 0){
            weightLR = X/(X+Y);
            weightFB = Y/(X+Y);
        }

        double dTheta = weightLR * dThetaLR + weightFB * dThetaFB;

        double dRobotX = B * sinXOverX(dTheta) + Q * Math.sin(dTheta) - L * cosXMinusOneOverX(dTheta) + P * (Math.cos(dTheta) - 1);
        double dRobotY = L * sinXOverX(dTheta) - P * Math.sin(dTheta) + B * cosXMinusOneOverX(dTheta) + Q * (Math.cos(dTheta) - 1);

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

    public RealMatrix getDeltaMatrix() {
        // gets deltas to be inputted into kalman filter
        return MatrixUtils.createRealMatrix(new double[][]{
                {dx},
                {dy},
                {dHeading}
        });
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("worldHeading: " + Math.toDegrees((worldHeadingRad)));

        return data;
    }

    public void setPose(Pose pose) {
        this.oldX = pose.x;
        this.oldY = pose.y;
        this.oldHeading = pose.heading;

        this.worldX = pose.x;
        this.worldY = pose.y;
        this.worldHeadingRad = pose.heading;
    }

    /**
     * Get the robot's current pose, containing x, y, and heading.
     * <p>
     * There is only one getter method to reduce the likelihood of values changing between getters.
     *
     * @return
     */
    public Pose getPose() {
        return new Pose(worldX, worldY, worldHeadingRad);
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