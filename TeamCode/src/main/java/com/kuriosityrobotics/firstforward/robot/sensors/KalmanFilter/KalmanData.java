package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import org.apache.commons.math3.linear.RealMatrix;

public class KalmanData {
    private final int dataType; // 0: odometry, // 1: vuforia
    private final RealMatrix data;

    public KalmanData(int dataType, RealMatrix data) {
        this.dataType = dataType;
        this.data = data;
    }

    public int getDataType() {
        return dataType;
    }

    public RealMatrix getData() {
        return data;
    }

    @Override
    public String toString() {
        return "KalmanData{" +
                "dataType=" + dataType +
                ", data=" + data +
                '}';
    }
}
