package com.kuriosityrobotics.firstforward.robot.sensors;


import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanData;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanGoodie;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanState;
import com.kuriosityrobotics.firstforward.robot.util.MatrixUtil;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.LinkedBlockingDeque;

/**
 * Extended Kalman Filter (EKF) for sensor fusion between odometry and vuforia Odometry is used as
 * prediction to generate estimate Vuforia is used as measurement to generate correction
 */
public class LocalizeKalmanFilter extends RollingVelocityCalculator implements Telemeter {

    // values
    private static final RealMatrix STARTING_COVARIANCE = MatrixUtils.createRealMatrix(new double[][]{
            {Math.pow(4, 2), 0, 0},
            {0, Math.pow(4, 2), 0},
            {0, 0, Math.pow(Math.toRadians(15), 2)}
    });
    private static final long KALMAN_WINDOW_SIZE_MS = 100;

    private final LinkedBlockingDeque<KalmanGoodie> unprocessedGoodieBag; // has null states
    private final LinkedList<KalmanGoodie> processedGoodieBag;

    private KalmanState state;

    protected LocalizeKalmanFilter(RealMatrix matrixPose) {
        synchronized (this) {
            state = new KalmanState(matrixPose, STARTING_COVARIANCE);
            unprocessedGoodieBag = new LinkedBlockingDeque<>();
            processedGoodieBag = new LinkedList<>();
            processedGoodieBag.add(new KalmanGoodie(null, SystemClock.elapsedRealtime(), new KalmanState(matrixPose, STARTING_COVARIANCE)));
        }
    }

    public static KalmanState prediction(KalmanState prev, KalmanData update) {
        RealMatrix[] prevMatrix = new RealMatrix[]{
                prev.getMean(), prev.getCov()
        };

        RealMatrix updateMatrix = update.getData();

        RealMatrix[] predictionMatrix = prediction(prevMatrix, updateMatrix);

        return new KalmanState(predictionMatrix[0], predictionMatrix[1]);
    }

    public static KalmanState correction(KalmanState pred, KalmanData obs) {
        RealMatrix[] predMatrix = new RealMatrix[]{
                pred.getMean(), pred.getCov()
        };

        RealMatrix obsMatrix = obs.getData();

        RealMatrix[] correctionMatrix = correction(predMatrix, obsMatrix);

        return new KalmanState(correctionMatrix[0], correctionMatrix[1]);
    }

    /**
     * Generates prediction based on odometry update
     *
     * @param prev:   the previous discrete-time step estimate Pose2D represented in a SimpleMatrix
     * @param update: the control update (odometry) Controls generalized as: Y distance, X distance,
     *                turn amount Controls are generalized from actual encoder updates (dY, dX,
     *                dHeading odo math generates)
     * @return prediction
     */
    public static RealMatrix[] prediction(RealMatrix[] prev, RealMatrix update) {
        // set up
        double prevX = prev[0].getEntry(0, 0);
        double prevY = prev[0].getEntry(1, 0);
        double prevHeading = angleWrap(prev[0].getEntry(2, 0));
        RealMatrix prevCov = prev[1];

        double odoDX = update.getEntry(0, 0);
        double odoDY = update.getEntry(1, 0);
        double odoDTheta = angleWrap(update.getEntry(2, 0));

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
                {Math.pow(0.8 * odoDX, 2), 0, 0},
                {0, Math.pow(0.8 * odoDY, 2), 0},
                {0, 0, Math.pow(0.8 * odoDTheta, 2)}
        }); // plugged in random values for now, would want to make acceleration (slip) based, this is basically error per update

        // calculate
        double predX = prevX + odoDX * Math.cos(prevHeading) + odoDY * Math.sin(prevHeading);
        double predY = prevY - odoDX * Math.sin(prevHeading) + odoDY * Math.cos(prevHeading);
        double predHeading = angleWrap(prevHeading + odoDTheta);

        //Log.v("kalman", "prediction -- x: " + predX + ", y: " + predY + ", heading: " + predHeading);


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
     *
     * @param pred: the prediction that needs to be correct Pose2D represented in SimpleMatrix
     * @param obs:  the observation that is used to correct (vuforia) column 1 is the actual tracker
     *              information (position on field) column 2 is where the robot is in the trackers
     *              coordinate system
     * @return corrected prediction
     */
    public static RealMatrix[] correction(RealMatrix[] pred, RealMatrix obs) {
        RealMatrix predCov = pred[1];

        RealMatrix predObs = pred[0]; // for full state sensor

        RealMatrix H = MatrixUtils.createRealMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });

        RealMatrix Q = MatrixUtils.createRealMatrix(new double[][]{
                {0.04, 0, 0},
                {0, 0.04, 0},
                {0, 0, 0.00121847}
        }); // random, somehow get stuff in here

        RealMatrix S = H.multiply(predCov).multiply(H.transpose()).add(Q);
        RealMatrix K = predCov.multiply(H.transpose().multiply(MatrixUtils.inverse(S)));

        RealMatrix error = obs.getColumnMatrix(0).subtract(predObs);
        error.setEntry(2, 0, angleWrap(error.getEntry(2, 0)));

        RealMatrix correct = pred[0].add(K.multiply(error));
        correct.setEntry(2, 0, angleWrap(correct.getEntry(2, 0)));
        RealMatrix correctCov = predCov.subtract(K.multiply(H).multiply(predCov));

        //Log.v("kalman", "correction -- x: " + correct.getEntry(0,0) + ", y: " + correct.getEntry(1,0) + ", heading: " + correct.getEntry(2,0));

        return new RealMatrix[]{correct, correctCov};
    }

    void update() {
        synchronized (this) {
            long currentTimeMillis = SystemClock.elapsedRealtime();

            unprocessedGoodieBag.drainTo(processedGoodieBag);

            processedGoodieBag.removeIf(n -> (n.getTimeStamp() > currentTimeMillis || n.getTimeStamp() < currentTimeMillis - KALMAN_WINDOW_SIZE_MS));
            processedGoodieBag.sort(Comparator.comparing(KalmanGoodie::getTimeStamp));

            if (processedGoodieBag.isEmpty())
                processedGoodieBag.add(new KalmanGoodie(null, currentTimeMillis, state));

            for (int i = 1; i < processedGoodieBag.size(); i++) {
                KalmanGoodie goodie = processedGoodieBag.get(i);
                KalmanGoodie prevGoodie = processedGoodieBag.get(i - 1);

                if (prevGoodie.isStateNull()) {
                    continue;
                }
                if (goodie.isDataNull()) {
                    continue;
                }

                KalmanState goodieState = prevGoodie.getState();
                KalmanData data = goodie.getData();

                if (data.getDataType() == 0) goodieState = prediction(goodieState, data);
                if (data.getDataType() == 1) goodieState = correction(goodieState, data);

                processedGoodieBag.set(i, new KalmanGoodie(goodie.getData(), goodie.getTimeStamp(), goodieState));

//                Log.v("kf", "procs: " + processedGoodieBag.getGoodie(i).toString());
            }

            KalmanGoodie lastGoodie = processedGoodieBag.peekLast();

            assert lastGoodie != null;
            state = lastGoodie.getState();

//            String s = "Processed Goodie Bag at time t = " + currentTimeMillis + "\n";
//            for (KalmanGoodie goodie : processedGoodieBag){
//                s += goodie + "\n";
//            }
//            Log.v("Kalman", "" + s);
        }

        calculateRollingVelocity(new PoseInstant(getPose(), SystemClock.elapsedRealtime() / 1000.0));
    }

    public void addGoodie(KalmanGoodie goodie) {
        unprocessedGoodieBag.add(goodie);
    }

    public void addGoodie(KalmanData data, long timeStamp) {
        unprocessedGoodieBag.add(new KalmanGoodie(data, timeStamp));
    }

    public Pose getPose() {
        synchronized (this) {
            RealMatrix matrixPose = state.getMean();

            double x = matrixPose.getEntry(0, 0);
            double y = matrixPose.getEntry(1, 0);
            double heading = matrixPose.getEntry(2, 0);

            return new Pose(x, y, heading);
        }
    }

    @Override
    public List<String> getTelemetryData() {
        synchronized (this) {
            ArrayList<String> data = new ArrayList<>();

            data.add("unprocessed: " + unprocessedGoodieBag.size() + " goodies");
            data.add("processed: " + processedGoodieBag.size() + " goodies");
            data.add("pose: " + MatrixUtil.toPoseString(state.getMean()));
            data.add("covar: " + MatrixUtil.toCovarianceString(state.getCov()));
            data.add("velo: " + getRollingVelocity().toString());

            return data;
        }
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        synchronized (this) {
            HashMap<String, Object> map = new HashMap<>();

            map.put("pose: ", MatrixUtil.toPoseString(state.getMean()));
            map.put("velo: ", getRollingVelocity().toString());

            return map;
        }
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
