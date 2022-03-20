package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import org.apache.commons.math3.linear.RealMatrix;

public class KalmanDatum {
    private final DatumType datumType; // 0: odometry, // 1: vuforia
    private final RealMatrix data;

    public KalmanDatum(DatumType datumType, RealMatrix data) {
        this.datumType = datumType;
        this.data = data;
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

    public enum DatumType {
        PREDICTION,
        CORRECTION
    }
}
