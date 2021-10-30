package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;

public class StateMachine {
    private State target;
    private boolean forceTarget = false;

    private long completeTime = 0;
    private State lastState;
    private State lastUpdateTarget;

    public void update() {
        long currentTime = SystemClock.elapsedRealtime();

        if (target == null) {
            return;
        }

        if (forceTarget || currentTime > completeTime) {
            if (lastUpdateTarget == null || lastUpdateTarget != target) {
                target.apply();

                completeTime = currentTime + target.getBlockDuration();
                lastState = lastUpdateTarget;
            }

            forceTarget = false;
        }

        if (currentTime > completeTime) {
            lastState = target;
        }

        lastUpdateTarget = target;
    }

    public void setCurrentState(State state) {
        this.target = state;
    }

    public State getCurrentState() {
        return lastState;
    }
}
