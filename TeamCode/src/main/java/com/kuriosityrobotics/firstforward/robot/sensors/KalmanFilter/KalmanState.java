package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import com.kuriosityrobotics.firstforward.robot.util.MatrixUtil;

import org.apache.commons.math3.linear.RealMatrix;

public class KalmanState {
    private RealMatrix mean;
    private RealMatrix cov;

    public KalmanState(RealMatrix mean, RealMatrix cov) {
        this.mean = mean;
        this.cov = cov;
    }

    public RealMatrix getMean() {
        return mean;
    }

    public RealMatrix getCov() {
        return cov;
    }

    public void setMean(RealMatrix mean) {
        this.mean = mean;
    }

    public void setCov(RealMatrix cov) {
        this.cov = cov;
    }

    @Override
    public String toString() {
        return "KalmanState {" +
                " mean = " + MatrixUtil.toPoseString(mean) +
                ", cov = " + MatrixUtil.toCovarianceString(cov) +
                "}";
    }
}
