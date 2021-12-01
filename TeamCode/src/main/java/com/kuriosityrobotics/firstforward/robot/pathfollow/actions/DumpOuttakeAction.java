package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class DumpOuttakeAction extends Action {
    private OuttakeModule.HopperDumpPosition dumpPosition;
    private boolean started = false;

    public DumpOuttakeAction(OuttakeModule.HopperDumpPosition dumpPosition) {
        this.dumpPosition = dumpPosition;
    }

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        if (!started) {
            robot.outtakeModule.dump(dumpPosition);
        } else if (robot.outtakeModule.getOuttakeState() == OuttakeModule.OuttakeState.IDLE) {
            this.completed = true;
        }

        started = true;
    }
}
