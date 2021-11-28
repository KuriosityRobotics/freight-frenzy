package com.kuriosityrobotics.firstforward.robot.modules.statemachine;

public class State {
    private final Runnable apply;
    private final long blockDuration;

    public State(Runnable apply, long blockDuration) {
        this.apply = apply;
        this.blockDuration = blockDuration;
    }

    public void apply() {
        this.apply.run();
    }

    public long getBlockDuration() {
        return blockDuration;
    }
}
