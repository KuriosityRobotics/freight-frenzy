package com.kuriosityrobotics.firstforward.robot.util.PID;

public class ClassicalPID {
    public final double P_FACTOR;
    public final double I_FACTOR;
    public final double D_FACTOR;

    private boolean reset;

    private double lastError;
    private double errorSum;
    private double errorChange;

    private long lastUpdateTime;

    /**
     * Constructs a ClassicalPIDController
     *
     * @param p
     * @param i
     * @param d
     */
    public ClassicalPID(double p, double i, double d) {
        P_FACTOR = p;
        I_FACTOR = i;
        D_FACTOR = d;

        this.reset = true;
        lastUpdateTime = NanoClock.now();
    }

    /**
     * Update the PID controller's scale. Should be called every iteration. Accumulates error onto
     * the integral.
     *
     * @param error The error between the current state and the desired state
     * @return Updated speed
     */
    public double calculateSpeed(double error) {
        error /= (NanoClock.now() - lastUpdateTime); // feature is back, now in nanos

        errorSum += error;

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

        lastUpdateTime = NanoClock.now();

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