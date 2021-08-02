package com.kuriosityrobotics.firstforward.robot.actions;

public class ActionsThread {
    /**
     * should never be called in teleop mode
     * @param tick
     */
    public void queueStateAuto(DesiredState tick) {
    }
    public void queueState(DesiredState tick) {}
}
