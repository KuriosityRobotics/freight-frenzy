package com.kuriosityrobotics.firstforward.robot.actions;

import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.qualcomm.robotcore.robot.Robot;
// todo: if pathfollow is just a fancy desiredstate then why is it in a differentp ackage and on a different thread :thonk:
// todo: absytract away state machine a bit maybe
public interface DesiredState extends Telemeter {
    /**
     * do whatever is needed to get to state
     * @param robot
     */
    void tick(Robot robot);

    /**
     * are we within an acceptable margin
     * @param robot
     * @return read above i'm not saying it again
     */
    boolean achieved(Robot robot);
}
