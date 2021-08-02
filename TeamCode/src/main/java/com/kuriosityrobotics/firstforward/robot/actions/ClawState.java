package com.kuriosityrobotics.firstforward.robot.actions;

import com.qualcomm.robotcore.robot.Robot;

// example
public class ClawState implements DesiredState {

    public void tick(Robot robot) {
    /*
    robot.getClaw().setMotorSpeed(pid stuff)
     */
    }

    public boolean achieved(Robot robot) {
        return true /*robot.getClaw().getMotorPosition() > 1*/;
    }
}
