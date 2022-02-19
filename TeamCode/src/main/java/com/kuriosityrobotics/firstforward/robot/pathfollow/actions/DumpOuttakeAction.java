package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.EXTEND;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class DumpOuttakeAction extends Action {
    private boolean raised = false;
    private boolean dumped = false;

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        if (!raised) {
            robot.outtakeModule.targetState = EXTEND;
            raised = robot.outtakeModule.atState(EXTEND);
        } else if (!dumped) {
            robot.outtakeModule.targetState = COLLAPSE;
            dumped = true;
        } else if (robot.outtakeModule.collapsed()) {
            this.completed = true;
        }

        if (this.completed)
            Log.v("outtake", "completed");
    }
}
