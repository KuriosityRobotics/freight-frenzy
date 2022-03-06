package com.kuriosityrobotics.firstforward.robot.sensors;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

public abstract class RollingVelocityCalculator {
    private static final double VELOCITY_WINDOW_S = 0.100;

    private final ConcurrentLinkedQueue<PoseInstant> history = new ConcurrentLinkedQueue<>();
    private Pose velocity = new Pose(0, 0, 0);

    @SuppressWarnings("ConstantConditions")
    protected void calculateRollingVelocity(PoseInstant currentPose) {
        double currentTime = SystemClock.elapsedRealtime() / 1000.0;

        // remove oldest velocities
        while (!history.isEmpty() && history.peek().timestamp < currentTime - VELOCITY_WINDOW_S) {
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

        if (velos.isEmpty()) {
            Log.e("MP", "No velocities!");
            return;
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
