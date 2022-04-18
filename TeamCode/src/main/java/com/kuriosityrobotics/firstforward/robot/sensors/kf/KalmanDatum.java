package com.kuriosityrobotics.firstforward.robot.sensors.kf;

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

    /**
     * @param stateToOutput For a prediction, this is the inverse of matrix G (for example, undoes rotation applied to convert odometry to global)
     *                      For a correction, this is matrix H
     */
    public KalmanDatum(long time, Primitive64Matrix mean, Primitive64Matrix covariance, Primitive64Matrix stateToOutput) {
        this.mean = mean;
        this.covariance = covariance;
        this.stateToOutput = stateToOutput;
        this.time = time;
    }

    public final Primitive64Matrix outputToState() {
        return stateToOutput.invert();
    }

    public Primitive64Matrix getStateToOutput() {
        return stateToOutput;
    }

    public Primitive64Matrix getMean() {
        return mean;
    }

    public Primitive64Matrix getCovariance() {
        return covariance;
    }

    public boolean isFullState() {
        return stateToOutput.isSquare()
                && stateToOutput.getRank() == stateToOutput.getMinDim();
    }

    @Override
    public String toString() {
        return "KalmanDatum{" +
                "time=" + time +
                ", mean=" + mean +
                ", covariance=" + covariance +
                ", stateToOutput=" + stateToOutput +
                '}';
    }
}
