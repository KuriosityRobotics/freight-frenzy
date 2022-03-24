package com.kuriosityrobotics.firstforward.robot.sensors.kf;

import static com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter.diagonal;

import android.os.SystemClock;

import org.ojalgo.matrix.Primitive64Matrix;

public class KalmanDatum {
    public final long time;
    protected final Primitive64Matrix mean;
    protected final Primitive64Matrix covariance;

    /**
     * For a prediction, this is the inverse of matrix G (for example, undoes rotation applied to convert odometry to global)
     * For a correction, this is matrix H
     */
    protected final Primitive64Matrix stateToOutput;

    public final Primitive64Matrix outputToState() {
        return stateToOutput.invert();
    }

    public final Primitive64Matrix stateToOutput() {
        return stateToOutput;
    }

    public Primitive64Matrix getStateToOutput() {
        return stateToOutput;
    }

    /**
     * @param stateToOutput For a prediction, this is the inverse of matrix G (for example, undoes rotation applied to convert odometry to global)
     * For a correction, this is matrix H
     */
    public KalmanDatum(long time, Primitive64Matrix mean, Primitive64Matrix covariance, Primitive64Matrix stateToOutput) {
        this.mean = mean;
        this.covariance = covariance;
        this.stateToOutput = stateToOutput;
        this.time = time;
    }

    /**
     * @param stateToOutput For a prediction, this is the inverse of matrix G (for example, undoes rotation applied to convert odometry to global)
     * For a correction, this is matrix H
     */
    public KalmanDatum(Primitive64Matrix mean, Primitive64Matrix covariance, Primitive64Matrix stateToOutput) {
        this(SystemClock.elapsedRealtime(), mean, covariance, stateToOutput);
    }
    public KalmanDatum(Primitive64Matrix mean, Primitive64Matrix covariance) {
        this(mean, covariance, Primitive64Matrix.FACTORY.makeIdentity(mean.getRowDim()));
    }

    /**
     * @param stateToOutput For a prediction, this is the inverse of matrix G (for example, undoes rotation applied to convert odometry to global)
     * For a correction, this is matrix H
     */
    public KalmanDatum(long time, Primitive64Matrix stateToOutput, double[] mean, double[] variance) {
        this(time, Primitive64Matrix.FACTORY.column(mean), diagonal(variance), stateToOutput);
    }

    public KalmanDatum(long time, double[] mean, double[] variance) {
        this(time, Primitive64Matrix.FACTORY.makeIdentity(mean.length), mean, variance);
    }

    public KalmanDatum(double[] mean, double[] variance) {
        this(SystemClock.elapsedRealtime(), Primitive64Matrix.FACTORY.column(mean), diagonal(variance), Primitive64Matrix.FACTORY.makeIdentity(mean.length));
    }

    public Primitive64Matrix getMean() {
        return mean;
    }

    public Primitive64Matrix getCovariance() {
        return covariance;
    }

    /**
     *     data are full-state iff their corresponding output matrix H is invertible (square & full rank)
     *      technically rows(H) == rows(kf mean), but if this is false and isFullState is true
     *      then matrix H is invalid anyway and we'll get an error when the kf tries to multiply it
     * THIS SHOULD ALWAYS BE TRUE FOR PREDICTIONS!
     */

    public boolean isFullState() {
        return stateToOutput.isSquare()
                && stateToOutput.getRank() == stateToOutput.getMinDim();
    }
}
