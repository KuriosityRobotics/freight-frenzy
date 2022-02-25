package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class ExtendOuttakeAction extends Action {
    OuttakeModule.VerticalSlideLevel slideLevel;
    OuttakeModule outtakeModule;

    public ExtendOuttakeAction(OuttakeModule outtakeModule, OuttakeModule.VerticalSlideLevel slideLevel) {
        this.outtakeModule = outtakeModule;
        this.slideLevel = slideLevel;
    }

    @Override
    public void tick() {
        super.tick();

        outtakeModule.targetSlideLevel = slideLevel;
        outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;

        this.completed = outtakeModule.atTargetState();
    }
}
