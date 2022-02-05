package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.IDLE;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.WAIT_FOR_COMMAND;
import static com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule.OuttakeState.WAIT_FOR_COMMAND2;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class DumpOuttakeAction extends Action {
    private OuttakeModule.HopperDumpPosition dumpPosition;
    private boolean started = false;

    public DumpOuttakeAction(OuttakeModule.HopperDumpPosition dumpPosition) {
        this.dumpPosition = dumpPosition;
    }

    boolean raised = false;
    boolean pivot = false;

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        if (!raised && robot.outtakeModule.getOuttakeState() != OuttakeModule.OuttakeState.WAIT_FOR_COMMAND2) {
            robot.outtakeModule.raise();
            raised = true;
        } else if(!pivot && (robot.outtakeModule.getOuttakeState() == WAIT_FOR_COMMAND || robot.outtakeModule.getOuttakeState() == IDLE)) {
            robot.outtakeModule.setSlideLevel(robot.visionThread.teamMarkerDetector.getLocation().slideLevel());
            robot.outtakeModule.pivotStraight();
            pivot = true;
        }
        else if (!this.started && robot.outtakeModule.getOuttakeState() == WAIT_FOR_COMMAND2) {
            robot.outtakeModule.dump(dumpPosition);
            this.started = true;

        } else if (pivot && raised && robot.outtakeModule.getOuttakeState() == OuttakeModule.OuttakeState.IDLE)
            this.completed = true;


        if (this.completed)
            Log.v("outtake", "" + completed);
    }
}
