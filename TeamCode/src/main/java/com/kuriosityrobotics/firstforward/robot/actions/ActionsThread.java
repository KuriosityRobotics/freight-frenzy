package com.kuriosityrobotics.firstforward.robot.actions;

import java.util.concurrent.LinkedBlockingQueue;

public class ActionsThread implements Runnable {
    // better solution than using .wait() and .notify() manually.
    private final LinkedBlockingQueue<DesiredState> stateQueue = new LinkedBlockingQueue<>();
    private final String configLocation;

    public ActionsThread(String configLocation) {
        this.configLocation = configLocation;
    }

    /**
     * should never be called in teleop mode
     */
    public void queueStateAuto(DesiredState state) {
        queueState(state);
    }

    public void queueState(DesiredState state) {
        try {
            stateQueue.put(state);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        while (!Thread.interrupted()) {
            try {
                var state = stateQueue.take();
                var timeSinceLastExecution = state.getLastExecutionTime().map( // almost a monad owo
                        lastExecutionTime -> System.nanoTime() - lastExecutionTime
                );

                if (!state.achieved()) {
                    state.tick(timeSinceLastExecution); // has side effects
                    stateQueue.put(state);
                }

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
