package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.EXTEND;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class DumpOuttakeAction extends Action {
    private boolean raised = false;
    private boolean dumped = false;
    private final OuttakeModule outtakeModule;

    public DumpOuttakeAction(OuttakeModule outtakeModule) {
        this.outtakeModule = outtakeModule;
    }


    @Override
    public void tick() {
        super.tick();

        if (!raised) {
            outtakeModule.targetState = EXTEND;
            raised = outtakeModule.atState(EXTEND);
        } else if (!dumped) {
            outtakeModule.targetState = COLLAPSE;
            dumped = true;
        } else if (outtakeModule.collapsed()) {
            this.completed = true;
        }

        if (this.completed)
            Log.v("outtake", "completed");
    }
}
