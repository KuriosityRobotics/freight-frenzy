package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.mean;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.math.MathUtil;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.stream.Collectors;

public class StallDetector {
    // constants
    private static final int STALL_DETECTOR_CAPACITY = 300;
    // TODO: tune (should be mostly accurate)
    private static final double STALL_EPSILON = 50;

    // data structures

    private final CircularFifoQueue<Pose> velocities = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> xMovs = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> yMovs = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> turnMovs = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);

    // states
    public volatile boolean isStalled;

    public void update(Pose velocity, double xMov, double yMov, double turnMov) {
        if (true)
            return;

        xMovs.add(xMov);
        yMovs.add(yMov);
        turnMovs.add(turnMov);
        velocities.add(velocity);

        if ((mean(velocities.stream().map(Pose::getX).collect(Collectors.toList()))) < 3 && mean(xMovs) > .2) {
            isStalled = true;
            return;
        }

        if ((mean(velocities.stream().map(Pose::getY).collect(Collectors.toList()))) < 3 && mean(yMovs) > .2) {
            isStalled = true;
            return;
        }
/*
        if ((mean(velocities.stream().map(Pose::getHeading).collect(Collectors.toList()))) < 3 && mean(turnMovs) > .2) {
            isStalled = true;
            return;
        }*/

    }

    public boolean isStalled() {
        return isStalled;
    }
}
