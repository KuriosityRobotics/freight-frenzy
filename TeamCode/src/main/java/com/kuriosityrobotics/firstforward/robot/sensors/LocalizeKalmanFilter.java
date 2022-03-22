package com.kuriosityrobotics.firstforward.robot.sensors;


import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum;
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
    private int vuforiaGoodiesProcessed = 0, odometryGoodiesProcessed = 0;

    protected LocalizeKalmanFilter(RealMatrix matrixPose) {
        synchronized (this) {
            state = new KalmanState(matrixPose, STARTING_COVARIANCE);
            unprocessedGoodieBag = new LinkedBlockingDeque<>();
            processedGoodieBag = new LinkedList<>();
            processedGoodieBag.add(new KalmanGoodie(null, SystemClock.elapsedRealtime(), new KalmanState(matrixPose, STARTING_COVARIANCE)));
        }
    }

    public static KalmanState prediction(KalmanState prev, KalmanDatum update) {
        var prioriMean = prev.getMean().getColumn(0);
        double prevX = prioriMean[0];
        double prevY = prioriMean[1];
        double prevHeading = prioriMean[2];

        var updateMatrix = update.getData().getColumn(0);
        RealMatrix prevCov = prev.getCov();

        double odoDX = updateMatrix[0];
        double odoDY = updateMatrix[1];
        double odoDTheta = updateMatrix[2];

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



        RealMatrix M = update.getCovariance(); // plugged in random values for now, would want to make acceleration (slip) based, this is basically error per update

        // calculate
        double predX = prevX + odoDX * Math.cos(prevHeading) + odoDY * Math.sin(prevHeading);
        double predY = prevY - odoDX * Math.sin(prevHeading) + odoDY * Math.cos(prevHeading);
        double predHeading = angleWrap(prevHeading + odoDTheta);

        //Log.v("kalman", "prediction -- x: " + predX + ", y: " + predY + ", heading: " + predHeading);


        RealMatrix predCov = G.multiply(prevCov).multiply(G.transpose()).add(V.multiply(M.multiply(V.transpose())));

        RealMatrix[] predictionMatrix = new RealMatrix[]{
                MatrixUtils.createRealMatrix(new double[][]{
                        {predX},
                        {predY},
                        {predHeading}
                }),
                predCov
        };

        return new KalmanState(predictionMatrix[0], predictionMatrix[1]);
    }

    public static KalmanState correction(KalmanState pred, KalmanDatum obs) {
        RealMatrix obsMatrix = obs.getData();
        if (obs.getOnly() != -1) {
            obsMatrix = pred.getMean().copy();
            obsMatrix.setEntry(obs.getOnly(), 0, obs.getData().getEntry(obs.getOnly(), 0));
        }

        RealMatrix predCov = pred.getCov();

        RealMatrix predObs = pred.getMean(); // for full state sensor

        RealMatrix H = MatrixUtils.createRealMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });

        RealMatrix Q = obs.getCovariance();

        RealMatrix S = H.multiply(predCov).multiply(H.transpose()).add(Q);
        RealMatrix K = predCov.multiply(H.transpose().multiply(MatrixUtils.inverse(S)));

        RealMatrix error = obsMatrix.getColumnMatrix(0).subtract(predObs);
        error.setEntry(2, 0, angleWrap(error.getEntry(2, 0)));

        RealMatrix correct = pred.getMean().add(K.multiply(error));
        correct.setEntry(2, 0, angleWrap(correct.getEntry(2, 0)));
        RealMatrix correctCov = predCov.subtract(K.multiply(H).multiply(predCov));

        if (obs.getOnly() != -1) {
            var row = correctCov.getRow(obs.getOnly());
            var col = correctCov.getColumn(obs.getOnly());

            correctCov = predCov.copy();
            correctCov.setRow(obs.getOnly(), row);
            correctCov.setColumn(obs.getOnly(), col);
        }
        //Log.v("kalman", "correction -- x: " + correct.getEntry(0,0) + ", y: " + correct.getEntry(1,0) + ", heading: " + correct.getEntry(2,0));

        RealMatrix[] correctionMatrix = new RealMatrix[]{correct, correctCov};

        return new KalmanState(correctionMatrix[0], correctionMatrix[1]);
    }

    void update() {
        synchronized (this) {
            long currentTimeMillis = SystemClock.elapsedRealtime();

            unprocessedGoodieBag.drainTo(processedGoodieBag);
            processedGoodieBag.sort(Comparator.comparing(KalmanGoodie::getTimeStamp));

            // remove all goodies too old or too new, but keep the latest goodie with a state.
            ArrayList<KalmanGoodie> toRemove = new ArrayList<>();
            KalmanGoodie lastStatedExpiringGoodie = null;
            for (KalmanGoodie goodie : processedGoodieBag) {
                if (goodie.getTimeStamp() > currentTimeMillis || goodie.getTimeStamp() < currentTimeMillis - KALMAN_WINDOW_SIZE_MS) {
                    if (!goodie.isStateNull()) { // it's a stated goodie
                        if (lastStatedExpiringGoodie != null) {
                            toRemove.add(lastStatedExpiringGoodie);
                        }
                        lastStatedExpiringGoodie = goodie;
                    } else {
                        toRemove.add(goodie);
                    }
                }
            }
            // remove the goodies marked for removal
            for (KalmanGoodie goodie : toRemove) {
                processedGoodieBag.remove(goodie);
            }

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
                KalmanDatum data = goodie.getData();

                switch (data.getDataType()) {
                    case PREDICTION:
                        odometryGoodiesProcessed++;
                        goodieState = prediction(goodieState, data);
                        break;
                    case CORRECTION:
                        vuforiaGoodiesProcessed++;
                        goodieState = correction(goodieState, data);
                        break;
                }

                goodie.setState(goodieState);

            }

            KalmanGoodie lastGoodie = processedGoodieBag.peekLast();
            assert lastGoodie != null;
            state = lastGoodie.getState();

            if (state == null) {
                Log.e("KF", "state nul! " + processedGoodieBag);
            }

        }

        calculateRollingVelocity(new PoseInstant(getPose(), SystemClock.elapsedRealtime() / 1000.0));
    }

    public void addGoodie(KalmanGoodie goodie) {
        unprocessedGoodieBag.add(goodie);
    }

    public void addGoodie(KalmanDatum data, long timeStamp) {
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

            data.add("vuf processed goodies:  " + vuforiaGoodiesProcessed);
            data.add("odo processed goodies:  " + odometryGoodiesProcessed);
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

    public static RealMatrix diagonal(double... values) {
        var matrix = new double[values.length][values.length];

        for (int i = 0; i < values.length; i++) {
            matrix[i][i] = values[i];
        }

        return MatrixUtils.createRealMatrix(matrix);
    }
}
