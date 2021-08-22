package com.kuriosityrobotics.firstforward.robot.modules;

import java.util.function.Function;

public abstract class StateMachine {
    public interface State {
        public Function execute();
        public long block_duration();
    }

    State target;
    boolean forceTarget = false;

    long completeTime = 0;
    State lastState;
    State lastUpdateTarget;

    public void update(long currentTime) {
        if (target == null) {
            return;
        }

        if (forceTarget || currentTime > completeTime) {
            if (lastUpdateTarget == null || lastUpdateTarget != target) {
                target.execute();

                completeTime = currentTime + target.block_duration();
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
