package com.kuriosityrobotics.firstforward.robot.sensors;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

public class Odometry implements Telemeter {
    // Encoders
    private final DcMotor yLeftEncoder;
    private final DcMotor yRightEncoder;
    private final DcMotor mecanumEncoder;

    // TODO: Position of the robot, This is how you manipulate the robot starting value
    private double worldX = 0;
    private double worldY = 0;
    private double worldHeadingRad = 0;

    // velocity of the robot
    private double xVel = 0;
    private double yVel = 0;
    private double angleVel = 0;

    // change in position of the robot
    private double dx = 0;
    private double dy = 0;
    private double dHeading = 0;

    // For position calculation
    private double lastLeftPosition = 0;
    private double lastRightPosition = 0;
    private double lastMecanumPosition = 0;

    // For velocity calculation
    private double oldX = 0;
    private double oldY = 0;
    private double oldHeading = 0;

    private double lastUpdateTime = 0;

    // Constants
    private final static double INCHES_PER_ENCODER_TICK = 0.0007284406721 * 100.0 / 101.9889;
    private final static double LR_ENCODER_DIST_FROM_CENTER = 6.942654509 * 3589.8638 / 3600.0 * 3531.4628211 / 3600.0;
    private final static double M_ENCODER_DIST_FROM_CENTER = 4.5;

    public Odometry(Robot robot) {
        robot.telemetryDump.registerTelemeter(this);

        yLeftEncoder = robot.hardwareMap.get(DcMotor.class, "fLeft");
        yRightEncoder = robot.hardwareMap.get(DcMotor.class, "fRight");
        mecanumEncoder = robot.hardwareMap.get(DcMotor.class, "bLeft");

//        // temporary stuff so I can get odo working on the robot at my house
//        yLeftEncoder = robot.hardwareMap.get(DcMotor.class, "leftodo");
//        yRightEncoder = robot.hardwareMap.get(DcMotor.class, "rightodo");
//        mecanumEncoder = robot.hardwareMap.get(DcMotor.class, "mecanumodo");

        resetEncoders();
    }

    public void update() {
        calculatePosition();

        calculateVelocity();
    }

    private void calculatePosition() {
        double newLeftPosition = -yLeftEncoder.getCurrentPosition();
        double newRightPosition = yRightEncoder.getCurrentPosition();
        double newMecanumPosition = mecanumEncoder.getCurrentPosition();

        double deltaLeftPosition = newLeftPosition - lastLeftPosition;
        double deltaRightPosition = newRightPosition - lastRightPosition;
        double deltaMecanumPosition = newMecanumPosition - lastMecanumPosition;

        updateWorldPosition(deltaLeftPosition, deltaRightPosition, deltaMecanumPosition);

        lastLeftPosition = newLeftPosition;
        lastRightPosition = newRightPosition;
        lastMecanumPosition = newMecanumPosition;
    }

    private void calculateVelocity() {
        long currentUpdateTime = SystemClock.elapsedRealtime();

        dx = (worldX - oldX) / (currentUpdateTime - lastUpdateTime);
        dy = (worldY - oldY) / (currentUpdateTime - lastUpdateTime);
        dHeading = (worldHeadingRad - oldHeading) / (currentUpdateTime - lastUpdateTime);

        xVel = 1000 * dx;
        yVel = 1000 * dy;
        angleVel = 1000 * dHeading;

        oldX = worldX;
        oldY = worldY;
        oldHeading = worldHeadingRad;

        lastUpdateTime = currentUpdateTime;
    }

    public void updateWorldPosition(double dLeftPod, double dRightPod, double dMecanumPod) {
        // convert all inputs to inches
        double dLeftPodInches = dLeftPod * INCHES_PER_ENCODER_TICK;
        double dRightPodInches = dRightPod * INCHES_PER_ENCODER_TICK;
        double dMecanumPodInches = dMecanumPod * INCHES_PER_ENCODER_TICK;

        // so its easier to type
        double L = dLeftPodInches;
        double R = dRightPodInches;
        double M = dMecanumPodInches;
        double P = LR_ENCODER_DIST_FROM_CENTER;
        double Q = M_ENCODER_DIST_FROM_CENTER;


        // find robot relative deltas
        double dTheta = (L - R) / (2 * P);
        double dRobotX = M * sinXOverX(dTheta) + Q * Math.sin(dTheta) - L * cosXMinusOneOverX(dTheta) + P * (Math.cos(dTheta) - 1);
        double dRobotY = L * sinXOverX(dTheta) - P * Math.sin(dTheta) + M * cosXMinusOneOverX(dTheta) + Q * (Math.cos(dTheta) - 1);

        worldX += dRobotX * Math.cos(worldHeadingRad) + dRobotY * Math.sin(worldHeadingRad);
        worldY += dRobotY * Math.cos(worldHeadingRad) - dRobotX * Math.sin(worldHeadingRad);
        //worldAngleRad =  (leftPodNewPosition - rightPodNewPosition) * INCHES_PER_ENCODER_TICK / (2 * P);
        worldHeadingRad += dTheta;
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
        mecanumEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeftPosition = 0;
        lastRightPosition = 0;
        lastMecanumPosition = 0;
    }

    public RealMatrix getDeltaMatrix() {
        // x needs to be negative for some reason
        return MatrixUtils.createRealMatrix(new double[][]{
                {dx},
                {dy},
                {dHeading}
        });
    }

    public RealMatrix getWorldMatrix() {
        return MatrixUtils.createRealMatrix(new double[][]{
                {worldX},
                {worldY},
                {worldHeadingRad}
        });
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("worldHeading: " + Math.toDegrees(worldHeadingRad));

        data.add("--");

        data.add("xVel: " + xVel);
        data.add("yVel: " + yVel);
        data.add("angleVel: " + angleVel);

        data.add("--");

        data.add("lastLeft: " + lastLeftPosition);
        data.add("lastRight: " + lastRightPosition);
        data.add("lastMecanum: " + lastMecanumPosition);

        return data;
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
