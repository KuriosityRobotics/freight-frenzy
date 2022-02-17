package com.kuriosityrobotics.firstforward.robot.modules;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.sd;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

import org.apache.commons.collections4.queue.CircularFifoQueue;

public class StallDetector {
    // constants
    private static final int STALL_DETECTOR_CAPACITY = 300;
    private static final double STALL_EPSILON = 0.1;

    // data structures
    private final CircularFifoQueue<Double> poseXSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> poseYSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);
    private final CircularFifoQueue<Double> poseHeadingSD = new CircularFifoQueue<>(STALL_DETECTOR_CAPACITY);

    // states
    private boolean isStalled;

    public void sendPose(Pose pose, double xMov, double yMov, double turnMov) {
        // add stuff
        poseXSD.add(pose.x);
        poseYSD.add(pose.y);
        poseHeadingSD.add(pose.heading);

        // do calculations
        // we strive for readability here
        boolean movementsNotSet = doublesEqual(xMov, 0) && doublesEqual(yMov, 0) && doublesEqual(turnMov, 0);

        boolean isXStalled = sd(poseXSD) < STALL_EPSILON;
        boolean isYStalled = sd(poseYSD) < STALL_EPSILON;
        boolean isHeadingStalled = sd(poseHeadingSD) < STALL_EPSILON;
        boolean movementsStalled = isXStalled && isYStalled && isHeadingStalled;

        isStalled = !movementsNotSet && movementsStalled;
    }

    public boolean isStalled() {
        return isStalled;
    }
}
