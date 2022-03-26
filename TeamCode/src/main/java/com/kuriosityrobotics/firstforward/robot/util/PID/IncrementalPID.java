package com.kuriosityrobotics.firstforward.robot.util.PID;

import android.os.SystemClock;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller that increments a scale using PID. This is different from a traditional PID
 * because the scale is incremented by a value determined using PID every cycle.
 */
public class IncrementalPID {
    private final double P_CONSTANT;
    private final double I_CONSTANT;
    private final double D_CONSTANT;

    private double scale;
    private boolean reset;

    private double lastError;
    private double errorSum;
    private long lastUpdateTime;

    private final double scaleMin;
    private final double scaleMax;

    /**
     * Constructs a VelocityPIDController with an initial scale of 1, a minimum scale of -1, and a
     * max of +1. This constructor is not recommended.
     *
     * @param P_CONSTANT
     * @param I_CONSTANT
     * @param D_CONSTANT
     */
    public IncrementalPID(double P_CONSTANT, double I_CONSTANT, double D_CONSTANT) {
        this(P_CONSTANT, I_CONSTANT, D_CONSTANT, 1);
    }

    /**
     * Constructs a VelocityPIDController with a scale minimum of -1 and a max of +1.
     *
     * @param P_CONSTANT
     * @param I_CONSTANT
     * @param D_CONSTANT
     * @param initialScale The starting scale of the controller.
     */
    public IncrementalPID(double P_CONSTANT, double I_CONSTANT, double D_CONSTANT, double initialScale) {
        this(P_CONSTANT, I_CONSTANT, D_CONSTANT, initialScale, -1, 1);
    }

    /**
     * Constructs a VelocityPIDController.
     *
     * @param P_CONSTANT
     * @param I_CONSTANT
     * @param D_CONSTANT
     * @param initialScale The starting scale of the controller.
     * @param scaleMin     The minimum possible value for the scale.
     * @param scaleMax     The maximum possible value for the scale.
     */
    public IncrementalPID(double P_CONSTANT, double I_CONSTANT, double D_CONSTANT, double initialScale, double scaleMin, double scaleMax) {
        this.P_CONSTANT = P_CONSTANT;
        this.I_CONSTANT = I_CONSTANT;
        this.D_CONSTANT = D_CONSTANT;

        this.scale = initialScale;

        this.lastUpdateTime = NanoClock.now();

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
        long timeDifference = (NanoClock.now() - lastUpdateTime);

        // error is now relative to how much time since there was last update; will accumulate less error
        error /= timeDifference;

        double p = error * this.P_CONSTANT;
        double i = 0;
        double d = 0;

        errorSum += error;

        if (!reset) {
            //update d to correct for overshoot
            d = this.D_CONSTANT * (error - lastError) / timeDifference;

            //update i accordingly
            i = errorSum * this.I_CONSTANT;
        } else {
            reset = false;
            i = errorSum * this.I_CONSTANT;
        }

        // debug capabilities
        //Log.v("BRAKING", "P: " + P + ", I:" + I + ", D: " + D);

        double increment = p + i + d;

        // more responsiveness
        scale = Range.clip(scale + increment, scaleMin, scaleMax);

        lastUpdateTime = NanoClock.now();
        errorSum = error;

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
