package com.kuriosityrobotics.firstforward.robot.modules.outtake;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

class ExtendOuttakeAction extends Action {
    private final OuttakeModule outtakeModule;
    private OuttakeModule.VerticalSlideLevel slideLevel;
    private OuttakeModule.TurretPosition turretPosition;

    public ExtendOuttakeAction(OuttakeModule outtakeModule, OuttakeModule.VerticalSlideLevel slideLevel, OuttakeModule.TurretPosition turretPosition) {
        this.outtakeModule = outtakeModule;
        this.slideLevel = slideLevel;
        this.turretPosition = turretPosition;
    }

    @Override
    public void tick() {
        super.tick();

        outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
        outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
        outtakeModule.targetTurret = turretPosition;
        outtakeModule.targetSlideLevel = slideLevel;
        outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;

        this.completed = outtakeModule.atTargetState();
    }
}
