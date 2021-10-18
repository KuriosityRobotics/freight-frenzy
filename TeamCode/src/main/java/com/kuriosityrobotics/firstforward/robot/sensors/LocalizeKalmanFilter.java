package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.math.MathFunctions.angleWrap;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.SimpleMatrixFormatter;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

/**
 * Extended Kalman Filter (EKF) for sensor fusion between odometry and vuforia
 * Odometry is used as prediction to generate estimate
 * Vuforia is used as measurement to generate correction
 */
public class LocalizeKalmanFilter implements KalmanFilter, Telemeter {

    public RealMatrix[] pose; // pose, cov

    // values
    private final RealMatrix STARTING_COVARIANCE = MatrixUtils.createRealMatrix(new double[][]{
            {Math.pow(0.125,2), 0, 0},
            {0, Math.pow(0.125,2), 0},
            {0, 0, Math.pow(Math.toRadians(2),2)}
    });

    public LocalizeKalmanFilter(Robot robot, RealMatrix pose){
        robot.telemetryDump.registerTelemeter(this);
        this.pose = new RealMatrix[]{pose, STARTING_COVARIANCE};
    }

    public LocalizeKalmanFilter(Robot robot, RealMatrix[] pose){
        robot.telemetryDump.registerTelemeter(this);
        this.pose = pose;
    }

    /**
     * Provides next estimate of discrete-time pose of robot based on update and/or observation
     * Smartly decides estimate method based on information given
     * @param update: the control update (odometry)
     *              Controls generalized as: Y distance, X distance, turn amount
     *              Controls are generalized from actual encoder updates (dY, dX, dHeading odo math generates)
     * @param obs: the observation that is used to correct (vuforia)
     *           column 1 is the actual tracker information (position on field)
     *           column 2 is where the robot is in the trackers coordinate system
     */
    public void update(RealMatrix update, RealMatrix obs){
        if (update != null && obs == null) pose = prediction(pose,update);
        else if (update == null && obs != null) pose = correction(pose, obs);
        else if (update != null && obs != null) pose = fuse(pose,update,obs);
    }

    /**
     * Generates prediction based on odometry update
     * @param prev: the previous discrete-time step estimate
     *            Pose2D represented in a SimpleMatrix
     * @param update: the control update (odometry)
     *              Controls generalized as: Y distance, X distance, turn amount
     *              Controls are generalized from actual encoder updates (dY, dX, dHeading odo math generates)
     * @return prediction
     */
    @Override
    public RealMatrix[] prediction(RealMatrix[] prev, RealMatrix update) {

        // set up
        double prevX = prev[0].getEntry(0,0);
        double prevY = prev[0].getEntry(1,0);
        double prevHeading = angleWrap(prev[0].getEntry(2,0));
        RealMatrix prevCov = prev[1];

        double odoDX = update.getEntry(0,0);
        double odoDY = update.getEntry(1,0);
        double odoDTheta = angleWrap(update.getEntry(2,0));

        RealMatrix G = MatrixUtils.createRealMatrix(new double[][]{
                {1, 0, -odoDX * Math.sin(prevHeading) + odoDY * Math.cos(prevHeading)},
                {0, 1, -odoDX * Math.cos(prevHeading) - odoDY * Math.sin(prevHeading)},
                {0, 0, 1}
        });

        RealMatrix V = MatrixUtils.createRealMatrix(new double[][]{
                {Math.cos(prevHeading), Math.sin(prevHeading), 0},
                {-Math.sin(prevHeading), Math.cos(prevHeading), 0},
                {0, 0, 1}
        });

        RealMatrix M = MatrixUtils.createRealMatrix(new double[][]{
                {0.0015 * Math.pow(odoDX,2), 0, 0},
                {0, 0.0015 * Math.pow(odoDY,2), 0},
                {0, 0, 0.00017 * Math.pow(odoDTheta,2)}
        }); // plugged in random values for now, would want to make acceleration (slip) based, this is basically error per update

        // calculate
        double predX = prevX + odoDX * Math.cos(prevHeading) + odoDY * Math.sin(prevHeading);
        double predY = prevY - odoDX * Math.sin(prevHeading) + odoDY * Math.cos(prevHeading);
        double predHeading = angleWrap(prevHeading + odoDTheta);
        RealMatrix predCov = G.multiply(prevCov).multiply(G.transpose()).add(V.multiply(M.multiply(V.transpose())));

        return new RealMatrix[]{
                MatrixUtils.createRealMatrix(new double[][]{
                        {predX},
                        {predY},
                        {predHeading}
                }),
                predCov
        };
    }

    /**
     * Generates correction based on vuforia tracker observation
     * @param pred: the prediction that needs to be correct
     *            Pose2D represented in SimpleMatrix
     * @param obs: the observation that is used to correct (vuforia)
     *           column 1 is the actual tracker information (position on field)
     *           column 2 is where the robot is in the trackers coordinate system
     * @return: corrected prediction
     */
    @Override
    public RealMatrix[] correction(RealMatrix[] pred, RealMatrix obs) {
        // set up

        // TODO: remember to package tracker data with tracker info in col 0 and observation in col 1
        double tX = obs.getEntry(0,0);
        double tY = obs.getEntry(1,0);
        double tPhi = angleWrap(obs.getEntry(2,0));

        double predX = pred[0].getEntry(0,0);
        double predY = pred[0].getEntry(1,0);
        double predHeading = angleWrap(pred[0].getEntry(2,0));
        RealMatrix predCov = pred[1];

        RealMatrix predObs = MatrixUtils.createRealMatrix(new double[][]{
                {(predX - tX) * Math.cos(tPhi) - (predY - tY) * Math.sin(tPhi)},
                {(predX - tX) * Math.sin(tPhi) + (predY - tY) * Math.cos(tPhi)},
                {predHeading - tPhi}
        });

        RealMatrix H = MatrixUtils.createRealMatrix(new double[][]{
                {Math.cos(tPhi), -Math.sin(tPhi), 0},
                {Math.sin(tPhi), Math.cos(tPhi), 0},
                {0, 0, 1}
        });

        RealMatrix Q = MatrixUtils.createRealMatrix(new double[][]{
                {0.04, 0, 0},
                {0, 0.04, 0},
                {0, 0, 0.00121847}
        }); // random, somehow get stuff in here

        RealMatrix S = H.multiply(predCov).multiply(H.transpose()).add(Q);
        RealMatrix K = predCov.multiply(H.transpose().multiply(MatrixUtils.inverse(S)));

        RealMatrix error = obs.getColumnMatrix(1).subtract(predObs);
        error.setEntry(2,0, angleWrap(error.getEntry(2,0)));

        RealMatrix correct = pred[0].add(K.multiply(error));
        correct.setEntry(2,0, angleWrap(correct.getEntry(2,0)));
        RealMatrix correctCov = predCov.subtract(K.multiply(H).multiply(predCov));

        return new RealMatrix[]{correct,correctCov};
    }

    /**
     * Generates prediction and corrects it
     * Simply runs prediction and correction back to back
     * @param prev: the previous discrete-time step estimate
     *            Pose2D represented in a SimpleMatrix
     * @param update: the control update (odometry)
     *              Controls generalized as: Y distance, X distance, turn amount
     *              Controls are generalized from actual encoder updates (dY, dX, dHeading odo math generates)
     * @param obs: the observation that is used to correct (vuforia)
     *           column 1 is the actual tracker information (position on field)
     *           column 2 is where the robot is in the trackers coordinate system
     * @return corrected prediction
     */
    @Override
    public RealMatrix[] fuse(RealMatrix[] prev, RealMatrix update, RealMatrix obs) {
        RealMatrix[] prediction = prediction(prev, update);
        RealMatrix[] correction = correction(prediction, obs);
        return correction;
    }

    @Override
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("pose: " + SimpleMatrixFormatter.toPoseString(pose[0]));
        data.add("STD: " + SimpleMatrixFormatter.toSTDString(pose[1]));

        return data;
    }

    @Override
    public String getName() {
        return "LocalizeKalmanFilter";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}