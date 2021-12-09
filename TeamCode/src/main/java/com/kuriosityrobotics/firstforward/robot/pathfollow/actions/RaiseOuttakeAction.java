package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class RaiseOuttakeAction extends Action {
    OuttakeModule.VerticalSlideLevel slideLevel;

    public RaiseOuttakeAction(OuttakeModule.VerticalSlideLevel slideLevel) {
        this.slideLevel = slideLevel;
    }

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        OuttakeModule.slideLevel = slideLevel;
        this.completed = true;
    }
}
