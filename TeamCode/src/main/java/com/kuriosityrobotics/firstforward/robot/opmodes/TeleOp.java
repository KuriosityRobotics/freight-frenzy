package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap, telemetry, this, true);
//            robot.resetPose(new Pose(28, 60, Math.toRadians(-90)));
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }

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

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    boolean wasSet = false;

    private void updateIntakeStates() {
        boolean setPower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON;

        if (setPower && !wasSet) {
            robot.intakeModule.retracted = false;
        }

        robot.intakeModule.intakePower = setPower && !robot.intakeModule.retracted
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
        }

        wasSet = setPower;
    }

    Button dpad_up = new Button();
    Button lBump = new Button();
    Button yGamepad2 = new Button();

    OuttakeModule.TurretPosition lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

    private void updateOuttakeStates() {
        if ((gamepad1.right_bumper || gamepad2.right_bumper) && robot.outtakeModule.targetState != OuttakeModule.OuttakeState.COLLAPSE)
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.COLLAPSE;

        boolean up = dpad_up.isSelected(gamepad2.dpad_up),
                right = gamepad2.dpad_right,
                left = gamepad2.dpad_left,
                y = yGamepad2.isSelected(gamepad2.y),
                x = gamepad2.x,
                b = gamepad2.b,
                lTrigger = gamepad2.left_trigger > 0;
        if (up || left || right) {
            robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
            robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;

            lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

            if (up) {
                if (robot.outtakeModule.targetState != OuttakeModule.OuttakeState.RAISE && robot.outtakeModule.targetState != OuttakeModule.OuttakeState.EXTEND) {
                    robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                } else {
                    if (robot.outtakeModule.targetSlideLevel == OuttakeModule.VerticalSlideLevel.TOP) {
                        robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP_TOP;
                    } else {
                        robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                    }
                }
            } else if (left || right) {
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.MID;
            }

            if (robot.outtakeModule.targetState != OuttakeModule.OuttakeState.EXTEND) {
                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.RAISE;
            }
        } else if (y) {
            if (robot.outtakeModule.targetState == OuttakeModule.OuttakeState.EXTEND) {
                if (lastTurretTarget == OuttakeModule.TurretPosition.STRAIGHT) {
                    lastTurretTarget = OuttakeModule.TurretPosition.ALLIANCE_LOCK;
                } else {
                    lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                }
            } else {
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

                robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;

                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
            }
        } else if (x || b) {
            robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
            robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.SHARED;

            if (x) {
                lastTurretTarget = OuttakeModule.TurretPosition.SHARED_LEFT;
            } else if (b) {
                lastTurretTarget = OuttakeModule.TurretPosition.SHARED_RIGHT;
            }
        } else if (gamepad2.dpad_down) {
            robot.outtakeModule.skipToCollapse();
        }


        if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_LEFT || lastTurretTarget == OuttakeModule.TurretPosition.SHARED_RIGHT) {
            if (lTrigger) {
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;

                if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_LEFT) {
                    robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_LEFT_MORE_EXTREME_ANGLE;
                } else if (lastTurretTarget == OuttakeModule.TurretPosition.SHARED_RIGHT) {
                    robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_RIGHT_MORE_EXTREME_ANGLE;
                }
            } else {
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
                robot.outtakeModule.targetTurret = lastTurretTarget;
            }
        } else {
            robot.outtakeModule.targetTurret = lastTurretTarget;
        }
    }

    private boolean capPicked = false;
    private boolean capLifted = false;
    private boolean capDropped = false;

    private void updateCapStates() {
        if (robot.outtakeModule.targetState == OuttakeModule.OuttakeState.COLLAPSE) {
            capPicked = false;
            capLifted = false;
            capDropped = false;
        }

        if (lBump.isSelected(gamepad2.left_bumper)) {
            if (!capPicked) {
                robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.CAP_PICKUP;
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.RETRACT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.DOWN;

                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;

                capPicked = true;
            } else if (!capLifted) {
                robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.CAP_DROP;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP;

                capLifted = true;
            } else if (!capDropped) {
                robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP_DROP;

                capDropped = true;
            } else {
                // done capping, collapse outtake and reset states to resume alliance hub
                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.COLLAPSE;

                robot.outtakeModule.targetPivot = OuttakeModule.PivotPosition.OUT;
                lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                robot.outtakeModule.targetLinkage = OuttakeModule.LinkagePosition.EXTEND;
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;

                capPicked = false;
                capLifted = false;
                capDropped = false;
            }
        }
    }

    private void updateCarouselStates() {
        robot.carouselModule.spin = gamepad2.right_stick_x > 0 || gamepad2.right_stick_x < 0;
    }
}
