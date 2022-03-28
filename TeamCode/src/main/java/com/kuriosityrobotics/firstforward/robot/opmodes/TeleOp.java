package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.opmodes.auto.AutoPaths;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();
    boolean wasSet = false;
    Button dpad_up = new Button();
    Button lBump = new Button();
    Button yGamepad2 = new Button();
    Button xG1 = new Button();
    OuttakeModule.TurretPosition lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
    private boolean capPicked = false;
    private boolean capLifted = false;
    private boolean capDropped = false;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, true);

        Robot.setBlue(true);
        robot.resetPose(new Pose(FULL_FIELD - 29.375, 64.5, Math.toRadians(90)));
        AutoPaths.calibrateVuforia(robot);
        robot.resetPose(new Pose(FULL_FIELD - 29.375, 64.5, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
            updateIntakeStates();
            updateOuttakeStates();
            updateCapStates();
            updateCarouselStates();
        }
    }

    private void updateDrivetrainStates() {
        double yMov = Math.signum(gamepad1.left_stick_y) * -Math.pow(gamepad1.left_stick_y, 2);
        double xMov = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double turnMov = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);

        if (gamepad1.left_bumper) {
            yMov /= 2;
            xMov /= 2;
            turnMov /= 2;
        }

        robot.getDrivetrain().setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        boolean setPower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON;

        if (setPower && !wasSet) {
            robot.getIntakeModule().retracted = false;
        }

        robot.getIntakeModule().intakePower = setPower && !robot.getIntakeModule().retracted
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.getIntakeModule().targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
        }

        wasSet = setPower;

        robot.getIntakeModule().enableAutoExtend = gamepad1.right_trigger < 0.15;
    }

    private void updateOuttakeStates() {
        if (xG1.isSelected(gamepad1.x)) {
            robot.getOuttakeModule().resetSlides();
        }

        if ((gamepad1.right_bumper || gamepad2.right_bumper) && robot.getOuttakeModule().targetState != OuttakeModule.OuttakeState.COLLAPSE)
            robot.getOuttakeModule().targetState = OuttakeModule.OuttakeState.COLLAPSE;

        boolean up = dpad_up.isSelected(gamepad2.dpad_up),
                right = gamepad2.dpad_right,
                left = gamepad2.dpad_left,
                y = yGamepad2.isSelected(gamepad2.y),
                x = gamepad2.x,
                b = gamepad2.b,
                lTrigger = gamepad2.left_trigger > 0;
        if (up || left || right) {
            robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
            robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.OUT;

            lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

            if (up) {
                if (robot.getOuttakeModule().targetState != OuttakeModule.OuttakeState.RAISE && robot.getOuttakeModule().targetState != OuttakeModule.OuttakeState.EXTEND) {
                    robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                } else {
                    if (robot.getOuttakeModule().targetSlideLevel == OuttakeModule.VerticalSlideLevel.TOP) {
                        robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP_TOP;
                    } else {
                        robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                    }
                }
            } else {
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.MID;
            }

            if (robot.getOuttakeModule().targetState != OuttakeModule.OuttakeState.EXTEND) {
                robot.getOuttakeModule().targetState = OuttakeModule.OuttakeState.RAISE;
            }
        } else if (y) {
            if (robot.getOuttakeModule().targetState == OuttakeModule.OuttakeState.EXTEND) {
                if (lastTurretTarget == OuttakeModule.TurretPosition.STRAIGHT) {
                    lastTurretTarget = OuttakeModule.TurretPosition.ALLIANCE_LOCK;
                } else {
                    lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                }
            } else {
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

                robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.OUT;
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;

                robot.getOuttakeModule().targetState = OuttakeModule.OuttakeState.EXTEND;
            }
        } else if (x || b) {
            robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
            robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.OUT;
            robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.SHARED;

            if (x) {
                lastTurretTarget = OuttakeModule.TurretPosition.SHARED_LEFT;
            } else {
                lastTurretTarget = OuttakeModule.TurretPosition.SHARED_RIGHT;
            }
        } else if (gamepad2.dpad_down) {
            robot.getOuttakeModule().skipToCollapse();
        }


        if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_LEFT || lastTurretTarget == OuttakeModule.TurretPosition.SHARED_RIGHT) {
            if (lTrigger) {
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;

                if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_LEFT) {
                    robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.SHARED_LEFT_MORE_EXTREME_ANGLE;
                } else if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_RIGHT) {
                    robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.SHARED_RIGHT_MORE_EXTREME_ANGLE;
                }
            } else {
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
                robot.getOuttakeModule().targetTurret = lastTurretTarget;
            }
        } else {
            robot.getOuttakeModule().targetTurret = lastTurretTarget;
        }
    }

    private void updateCapStates() {
        if (robot.getOuttakeModule().targetState == OuttakeModule.OuttakeState.COLLAPSE) {
            capPicked = false;
            capLifted = false;
            capDropped = false;
        }

        if (lBump.isSelected(gamepad2.left_bumper)) {
            if (!capPicked) {
                robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.CAP_PICKUP;
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.DOWN;

                robot.getOuttakeModule().targetState = OuttakeModule.OuttakeState.EXTEND;

                capPicked = true;
            } else if (!capLifted) {
                robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.CAP_DROP;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP;

                capLifted = true;
            } else if (!capDropped) {
                robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.OUT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP_DROP;

                capDropped = true;
            } else {
                // done capping, collapse outtake and reset states to resume alliance hub
                robot.getOuttakeModule().targetState = OuttakeModule.OuttakeState.COLLAPSE;

                robot.getOuttakeModule().targetPivot = OuttakeModule.PivotPosition.OUT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.getOuttakeModule().targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.getOuttakeModule().targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;

                capPicked = false;
                capLifted = false;
                capDropped = false;
            }
        }
    }

    private void updateCarouselStates() {
        robot.getCarouselModule().setSpin(gamepad2.right_stick_x > 0 || gamepad2.right_stick_x < 0);
    }
}
