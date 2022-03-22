package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import org.apache.commons.math3.linear.RealMatrix;

public class KalmanDatum {
    private final DatumType datumType; // 0: odometry, // 1: vuforia
    private final RealMatrix data, covariance;

    private final int only;

    public KalmanDatum(DatumType datumType, RealMatrix data, RealMatrix covariance) {
        this(datumType, data, covariance, -1);
    }

    public KalmanDatum(DatumType datumType, RealMatrix data, RealMatrix covariance, int only) {
        this.datumType = datumType;
        this.data = data;
        this.covariance = covariance;
        this.only = only;
    }

    public DatumType getDataType() {
        return datumType;
    }

    public RealMatrix getData() {
        return data;
    }

    @Override
    public String toString() {
        return "KalmanDatum{" +
                "datumType=" + datumType +
                ", data=" + data +
                '}';
    }

    public int getOnly() {
        return only;
    }

    public RealMatrix getCovariance() {
        return covariance;
    }

    public enum DatumType {
        PREDICTION,
        CORRECTION
    }
}
