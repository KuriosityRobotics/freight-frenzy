package com.kuriosityrobotics.firstforward.robot.modules.outtake;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

class ExtendOuttakeAction extends Action {
    private final OuttakeModule outtakeModule;
    protected OuttakeModule.VerticalSlideLevel slideLevel;

    public ExtendOuttakeAction(OuttakeModule outtakeModule, OuttakeModule.VerticalSlideLevel slideLevel) {
        this.outtakeModule = outtakeModule;
        this.slideLevel = slideLevel;
    }

    @Override
    public void tick() {
        super.tick();

        outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
        outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
        outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
        outtakeModule.targetSlideLevel = slideLevel;
        outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;

        this.completed = outtakeModule.atTargetState();
    }
}
