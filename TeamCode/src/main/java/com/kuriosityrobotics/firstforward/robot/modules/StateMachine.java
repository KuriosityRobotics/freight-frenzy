package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;

public class StateMachine {
    public interface State {
        public void execute();

        public long blockDuration();
    }

    State target;
    boolean forceTarget = false;

    long completeTime = 0;
    State lastState;
    State lastUpdateTarget;

    public void update() {
        long currentTime = SystemClock.elapsedRealtime();

        if (target == null) {
            return;
        }

        if (forceTarget || currentTime > completeTime) {
            if (lastUpdateTarget == null || lastUpdateTarget != target) {
                target.execute();

                completeTime = currentTime + target.blockDuration();
                lastState = lastUpdateTarget;
            }

            forceTarget = false;
        }

        if (currentTime > completeTime) {
            lastState = target;
        }

        lastUpdateTarget = target;
    }

    public State getCurrentState() {
        return lastState;
    }
}
