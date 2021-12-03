package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

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

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        if (!started) {
            robot.outtakeModule.dump(dumpPosition);
        } else if (robot.outtakeModule.getOuttakeState() == OuttakeModule.OuttakeState.SLIDES_DOWN) {
            this.completed = true;
            Log.v("dump outtake", "DONE!");
        }

        started = true;
    }
}
