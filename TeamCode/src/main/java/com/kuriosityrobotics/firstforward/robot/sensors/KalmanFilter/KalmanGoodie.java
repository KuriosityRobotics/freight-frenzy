package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import org.apache.commons.math3.linear.RealMatrix;

public class KalmanGoodie {
    private KalmanData data;
    private long timeStamp;

    private KalmanState state; // current

    public KalmanGoodie(KalmanData data, long timeStamp, KalmanState state) {
        this.data = data;
        this.timeStamp = timeStamp;
        this.state = state;
    }

    public KalmanGoodie(KalmanData data, long timeStamp){
        this.data = data;
        this.timeStamp = timeStamp;
        state = null;
    }

    public boolean isDataNull(){
        return data == null;
    }

    public boolean isStateNull(){
        return state == null;
    }

    public void setState(KalmanState state) {
        this.state = state;
    }

    public KalmanData getData() {
        return data;
    }

    public long getTimeStamp() {
        return timeStamp;
    }

    public KalmanState getState() {
        return state;
    }


    public void setMean(RealMatrix mean){
        state.setMean(mean);
    }

    public void setCov(RealMatrix cov){
        state.setCov(cov);
    }

    public RealMatrix getMean(){
        if (state == null) return null;
        return state.getMean();
    }

    public RealMatrix getCov(){
        if (state == null) return null;
        return state.getCov();
    }
}
