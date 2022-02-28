package com.kuriosityrobotics.firstforward.robot.util.PID;

import android.util.Log;

public class IThresholdPID {
    public final double P_FACTOR;
    public final double I_FACTOR;
    public final double D_FACTOR;
    public final double iThreshold;

    private boolean reset;

    private double lastError;
    private double errorSum;
    private double errorChange;

    /**
     * Constructs a ClassicalPIDController
     *
     * @param p
     * @param i
     * @param d
     */
    public IThresholdPID(double p, double i, double d, double iThreshold) {
        P_FACTOR = p;
        I_FACTOR = i;
        D_FACTOR = d;
        this.iThreshold = iThreshold;

        this.reset = true;
    }

    /**
     * Update the PID controller's scale. Should be called every iteration. Accumulates error onto
     * the integral.
     *
     * @param error The error between the current state and the desired state
     * @return Updated speed
     */
    public double calculateSpeed(double error) {
        if (Math.abs(error) < iThreshold) {
            errorSum += error;
        } else {
            errorSum = 0;
        }

        double p = error * P_FACTOR;
        double i = 0;
        double d = 0;

        if (!reset) {
            //update d to correct for overshoot
            d = D_FACTOR * (error - lastError);
        } else {
            reset = false;
            errorSum = error;
            d = 0;
        }

        //update i accordingly
        i = errorSum * I_FACTOR;

        lastError = error;
        errorChange = d;

        return p + i + d;
    }

    /**
     * Reset the PID controller using given default scale
     */

    public void reset() {
        reset = true;
        errorSum = 0;
    }

    public double getD() { return errorChange; }
}