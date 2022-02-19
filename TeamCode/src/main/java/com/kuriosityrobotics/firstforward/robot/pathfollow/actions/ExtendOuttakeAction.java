package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class ExtendOuttakeAction extends Action {
    OuttakeModule.VerticalSlideLevel slideLevel;

    public ExtendOuttakeAction(OuttakeModule.VerticalSlideLevel slideLevel) {
        this.slideLevel = slideLevel;
    }

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        robot.outtakeModule.targetSlideLevel = slideLevel;
        robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;

        this.completed = robot.outtakeModule.atTargetState();
    }
}
