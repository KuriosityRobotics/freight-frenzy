package com.kuriosityrobotics.firstforward.robot.sensors;

import org.apache.commons.math3.linear.RealMatrix;

public interface KalmanFilter {
    RealMatrix[] prediction(RealMatrix[] prev, RealMatrix update);
    RealMatrix[] correction(RealMatrix[] pred, RealMatrix obs);
    RealMatrix[] fuse(RealMatrix[] prev, RealMatrix update, RealMatrix obs);
}