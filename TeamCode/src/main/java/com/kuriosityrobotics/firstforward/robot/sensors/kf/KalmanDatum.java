package com.kuriosityrobotics.firstforward.robot.sensors.kf;

import static com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter.diagonal;

import org.ojalgo.matrix.Primitive64Matrix;

import java.time.Instant;

public class KalmanDatum {
    public final Instant time;
    protected final Primitive64Matrix mean;
    protected final Primitive64Matrix covariance;
    protected final Primitive64Matrix stateToOutput;

    public Primitive64Matrix getStateToOutput() {
        return stateToOutput;
    }

    public KalmanDatum(Instant time, Primitive64Matrix mean, Primitive64Matrix covariance, Primitive64Matrix stateToOutput) {
        this.mean = mean;
        this.covariance = covariance;
        this.stateToOutput = stateToOutput;
        this.time = time;
    }
    public KalmanDatum(Primitive64Matrix mean, Primitive64Matrix covariance, Primitive64Matrix stateToOutput) {
        this(Instant.now(), mean, covariance, stateToOutput);
    }
    public KalmanDatum(Primitive64Matrix mean, Primitive64Matrix covariance) {
        this(mean, covariance, Primitive64Matrix.FACTORY.makeIdentity(mean.getRowDim()));
    }

    public KalmanDatum(Instant time, Primitive64Matrix stateToOutput, double[] mean, double[] variance) {
        this(time, Primitive64Matrix.FACTORY.column(mean), diagonal(variance), stateToOutput);
    }

    public KalmanDatum(Instant time, double[] mean, double[] variance) {
        this(time, Primitive64Matrix.FACTORY.makeIdentity(mean.length), mean, variance);
    }

    public KalmanDatum(double[] mean, double[] variance) {
        this(Instant.now(), Primitive64Matrix.FACTORY.column(mean), diagonal(variance), Primitive64Matrix.FACTORY.makeIdentity(mean.length));
    }

    public Primitive64Matrix getMean() {
        return mean;
    }

    public Primitive64Matrix getCovariance() {
        return covariance;
    }

    // data are full-state iff their corresponding output matrix H is invertible (square & full rank)
    // technically rows(H) == rows(kf mean), but if this is false and isFullState is true
    // then matrix H is invalid anyway and we'll get an error when the kf tries to multiply it
    public boolean isFullState() {
        return stateToOutput.isSquare()
                && stateToOutput.getRank() == stateToOutput.getMinDim();
    }
}
