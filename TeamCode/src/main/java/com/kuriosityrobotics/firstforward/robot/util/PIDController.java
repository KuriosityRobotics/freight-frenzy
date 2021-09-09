package com.kuriosityrobotics.firstforward.robot.util;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller that increments a scale using PID. This is different from a traditional PID
 * because the scale is incremented by a value determined using PID every cycle.
 */
public class PIDController {
    private final double p;
    private final double i;
    private final double d;

    private double scale;
    private boolean reset;

    private double lastError;
    private double errorSum;
    private double lastUpdateTime;

    private final double scaleMin;
    private final double scaleMax;

    /**
     * Constructs a VelocityPIDController with an initial scale of 1, a minimum scale of -1, and a
     * max of +1. This constructor is not recommended.
     *
     * @param p
     * @param i
     * @param d
     */
    public PIDController(double p, double i, double d) {
        this(p, i, d, 1);
    }

    /**
     * Constructs a VelocityPIDController with a scale minimum of -1 and a max of +1.
     *
     * @param p
     * @param i
     * @param d
     * @param initialScale The starting scale of the controller.
     */
    public PIDController(double p, double i, double d, double initialScale) {
        this(p, i, d, initialScale, -1, 1);
    }

    /**
     * Constructs a VelocityPIDController.
     *
     * @param p
     * @param i
     * @param d
     * @param initialScale The starting scale of the controller.
     * @param scaleMin     The minimum possible value for the scale.
     * @param scaleMax     The maximum possible value for the scale.
     */
    public PIDController(double p, double i, double d, double initialScale, double scaleMin, double scaleMax) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.scale = initialScale;

        this.lastUpdateTime = SystemClock.elapsedRealtime();

        this.scaleMin = scaleMin;
        this.scaleMax = scaleMax;

        this.reset = true;
    }

    /**
     * Update the PID controller's scale. Should be called every iteration. Accumulates error onto
     * the integral.
     *
     * @param error The error between the current state and the desired state
     * @return Updated PID scale
     */
    public double calculateScale(double error) {
        long currentTime = SystemClock.elapsedRealtime();
        long timeDifference = (long) (currentTime - lastUpdateTime);

        // error is now relative to how much time since there was last update; will accumulate less error
        error /= timeDifference;

        double P = error * p;
        double I = 0;
        double D = 0;

        if (!reset) {
            //update d to correct for overshoot
            D = d * (error - lastError) / timeDifference;

            errorSum += error;
            //update i accordingly
            I = errorSum * i;
        } else {
            reset = false;
        }

        // debug capabilities
        Log.v("BRAKING", "P: " + P + ", I:" + I + ", D: " + D);

        double increment = P + I + D;

        // more responsiveness
        scale = Range.clip(scale + increment, scaleMin, scaleMax);

        lastUpdateTime = currentTime;
        lastError = error;

        return scale;
    }

    /**
     * Reset the PID controller using given default scale
     */
    public void reset(double scale) {
        reset = true;
        errorSum = 0;

        this.scale = scale;
    }
}
