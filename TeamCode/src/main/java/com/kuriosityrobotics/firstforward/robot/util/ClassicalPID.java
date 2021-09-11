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

    private double speed;
    private boolean reset;

    private double lastError;
    private double errorSum;
    private double lastUpdateTime;

    private final double minSpeed;
    private final double maxSpeed;

    /**
     * Constructs a VelocityPIDController with an initial scale of 1, a minimum scale of -1, and a
     * max of +1. This constructor is not recommended.
     *
     * @param p
     * @param i
     * @param d
     */
    public ClassicalPID(double p, double i, double d) {
        this(p, i, d, 1);
    }

    /**
     * Constructs a VelocityPIDController with a scale minimum of -1 and a max of +1.
     *
     * @param p
     * @param i
     * @param d
     * @param initialSpeed The starting scale of the controller.
     */
    public ClassicalPID(double p, double i, double d, double initialSpeed) {
        this(p, i, d, initialSpeed, -1, 1);
    }

    /**
     * Constructs a VelocityPIDController.
     *
     * @param p
     * @param i
     * @param d
     * @param initialSpeed The starting scale of the controller.
     * @param minSpeed     The minimum possible value for the scale.
     * @param maxSpeed     The maximum possible value for the scale.
     */
    public ClassicalPID(double p, double i, double d, double initialSpeed, double minSpeed, double maxSpeed) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.speed = initialSpeed;

        this.lastUpdateTime = SystemClock.elapsedRealtime();

        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;

        this.reset = true;
    }

    /**
     * Update the PID controller's scale. Should be called every iteration. Accumulates error onto
     * the integral.
     *
     * @param error The error between the current state and the desired state
     * @return Updated PID scale
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

        double robotspeed = p + i + d;

        // more responsiveness
        this.speed = Range.clip(robotspeed, minSpeed, maxSpeed);

        lastUpdateTime = currentTime;

        return this.speed;
    }

    /**
     * Reset the PID controller using given default scale
     */
    public void reset(double scale) {
        reset = true;
        errorSum = 0;

        this.speed = scale;
    }
}
