package com.kuriosityrobotics.firstforward.robot.modules;

public class State {
    private final Runnable apply;
    private final long blockDuration;

    private State(Runnable apply, long blockDuration) {
        this.apply = apply;
        this.blockDuration = blockDuration;
    }

    public static State State(Runnable apply, long blockDuration) {
        return new State(apply, blockDuration);
    }

    public void apply() {
        this.apply.run();
    }

    public long getBlockDuration() {
        return blockDuration;
    }
}
