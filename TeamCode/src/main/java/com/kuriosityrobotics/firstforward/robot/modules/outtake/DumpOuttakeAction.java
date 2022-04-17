package com.kuriosityrobotics.firstforward.robot.modules.outtake;

import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.COLLAPSE;
import static com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule.OuttakeState.DUMP;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

class DumpOuttakeAction extends Action {
    private final OuttakeModule outtakeModule;

    public DumpOuttakeAction(OuttakeModule outtakeModule) {
        this.outtakeModule = outtakeModule;
    }

    @Override
    public void tick() {
        super.tick();

        outtakeModule.targetState = DUMP;

        if (outtakeModule.executingState(DUMP)) {
            completed = true;
            outtakeModule.targetState = COLLAPSE;
        }
    }
}
