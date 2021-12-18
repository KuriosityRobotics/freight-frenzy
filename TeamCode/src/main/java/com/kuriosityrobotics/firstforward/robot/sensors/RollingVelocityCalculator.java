package com.kuriosityrobotics.firstforward.robot.sensors;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.math.Pose;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

public abstract class RollingVelocityCalculator {
    private static final double VELOCITY_WINDOW_S = 0.100;

    public ConcurrentLinkedQueue<PoseInstant> history = new ConcurrentLinkedQueue<>();
    private Pose velocity = new Pose(0, 0, 0);

    protected void calculateRollingVelocity(PoseInstant currentPose) {
        double currentTime = SystemClock.elapsedRealtime() / 1000.0;

        // remove oldest velocities
        while (history.peek() != null && history.peek().timestamp < currentTime - VELOCITY_WINDOW_S) {
            history.poll();
        }

        // add current pose
        history.add(currentPose);

        ArrayList<Pose> velos = new ArrayList<>();

        // calculate the velocity between each datapoint
        Iterator<PoseInstant> itr = history.iterator();
        PoseInstant last = itr.next();
        while (itr.hasNext()) {
            PoseInstant current = itr.next();

            if (current.timestamp == last.timestamp) {
                continue;
            }

            velos.add(current.difference(last).divide(current.timestamp - last.timestamp));

            last = current;
        }

        Pose sum = new Pose(0, 0, 0);
        // avg the calculated velos
        for (Pose pose : velos) {
            sum = sum.add(pose);
        }
        Pose avg = sum.divide(velos.size());

        this.velocity = avg;
    }

    public Pose getRollingVelocity() {
        return this.velocity;
    }
}
