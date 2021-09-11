package com.kuriosityrobotics.firstforward.robot.util;

import android.os.SystemClock;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller that increments a scale using PID. This is different from a traditional PID
 * because the scale is incremented by a value determined using PID every cycle.
 */
public class ClassicalPID {
    private final double p;
    private final double i;
    private final double d;

    private boolean reset;

    private double lastError;
    private double errorSum;
    private double lastUpdateTime;

    /**
     * Constructs a ClassicalPIDController
     *
     * @param p
     * @param i
     * @param d
     */
    public ClassicalPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.lastUpdateTime = SystemClock.elapsedRealtime();

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
        long currentTime = SystemClock.elapsedRealtime();
        long timeDifference = (long) (currentTime - lastUpdateTime);

        // error is now relative to how much time since there was last update; will accumulate less error
        error /= timeDifference;

        errorSum += error;

        double p = error * this.p;
        double i = 0;
        double d = 0;

        if (!reset) {
            //update d to correct for overshoot
            d = this.d * (error - lastError);
        } else {
            reset = false;
            errorSum = error;
            d = 0;
        }

        //update i accordingly
        i = errorSum * this.i;

        lastUpdateTime = currentTime;

        double robotspeed = p + i + d;

        return robotspeed;
    }

    /**
     * Reset the PID controller using given default scale
     */
    public void reset() {
        reset = true;
        errorSum = 0;
    }
}