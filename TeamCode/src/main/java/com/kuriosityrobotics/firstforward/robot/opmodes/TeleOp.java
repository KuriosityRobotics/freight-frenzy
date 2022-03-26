package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock.AngleLockType.LOCK;
import static com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock.AngleLockType.NO_LOCK;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.closestNum;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();
    private static final Point BLUE_GOAL_ENTRANCE = new Point(FULL_FIELD - 7, 48);
    private static final Point RED_GOAL_ENTRANCE = new Point(7, 48);

    boolean isRed;
    PurePursuit wallridePursuit = null;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, true);
        isRed = robot.sensorThread.getPose().isInRange(0, 0, 70, 140);
        robot.sensorThread.resetPose(new Pose(6.5, 70, Math.PI / 2));
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

    public boolean isFollowingWall() {
        return wallridePursuit != null;
    }

    private boolean isInGoal() {
        return robot.sensorThread.getPose().y < 48;
    }

    private Point getClosestEntrance() {
        Pose pose = robot.sensorThread.getPose();
        double distanceToBlueEntrance = pose.distance(BLUE_GOAL_ENTRANCE);
        double distanceToRedEntrance = pose.distance(RED_GOAL_ENTRANCE);

        if (distanceToBlueEntrance < distanceToRedEntrance)
            return BLUE_GOAL_ENTRANCE;
        else
            return RED_GOAL_ENTRANCE;
    }

    private void startFollowingWall() {
        WayPoint start = new WayPoint(robot.sensorThread.getPose());

        Pose pivotPose = new Pose(
                start, closestNum(robot.sensorThread.getPose().heading, new double[]{0, Math.PI})
        );
        WayPoint pivot = new WayPoint(pivotPose);

        Pose entrancePose = new Pose(getClosestEntrance(), pivotPose.heading);
        WayPoint entrance = new WayPoint(entrancePose.x, entrancePose.y, new AngleLock(
                LOCK, entrancePose.heading
        ));

        WayPoint insideGoal = new WayPoint(
                entrancePose.x,
                36,
                new AngleLock(isInGoal() ? LOCK : NO_LOCK, entrancePose.heading)
        );

        WayPoint[] path = isInGoal() ?
                new WayPoint[]{start, pivot, insideGoal, entrance} :
                new WayPoint[]{start, pivot, entrance, insideGoal};

        wallridePursuit = new PurePursuit(path, 0.2);
    }

    private void stopFollowingWall() {
        robot.telemetryDump.removeTelemeter(wallridePursuit);
        wallridePursuit = null;
    }

    private boolean inputMovementsZero() {
        return doublesEqual(gamepad1.left_stick_x, 0) &&
                doublesEqual(gamepad1.left_stick_y, 0);
    }


    private void updateDrivetrainStates() {
        double yMov = Math.signum(gamepad1.left_stick_y) * -Math.pow(gamepad1.left_stick_y, 2);
        double xMov = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double turnMov = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            if (inSlowMode()) {
                yMov /= 2;
                xMov /= 2;
                turnMov /= 2;
            }

            if (gamepad1.x && !isFollowingWall())
                startFollowingWall();

            if (isFollowingWall()) {
                wallridePursuit.update(robot, robot.drivetrain);
                if (wallridePursuit.atEnd(robot) || !inputMovementsZero())
                    stopFollowingWall();
            } else
                setDrivetrainMovements(yMov, xMov, turnMov);
        }
    }

    private boolean inSlowMode() {
        return gamepad1.right_bumper;
    }

    private void setDrivetrainMovements(double yMov, double xMov, double turnMov) {
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

        robot.intakeModule.enableAutoExtend = gamepad1.right_trigger < 0.15;
    }

    Button dpad_up = new Button();
    Button lBump = new Button();
    Button yGamepad2 = new Button();
    Button xG1 = new Button();

    OuttakeModule.TurretPosition lastTurretTarget = OuttakeModule.TurretPosition.STRAIGHT;

    private void updateOuttakeStates() {
        if (xG1.isSelected(gamepad1.x)) {
            robot.outtakeModule.resetSlides();
        }

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
