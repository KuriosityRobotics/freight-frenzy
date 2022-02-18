package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.closestNum;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;
import static com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock.AngleLockType.LOCK;
import static com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock.AngleLockType.NO_LOCK;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

    private static final Point BLUE_GOAL_ENTRANCE = new Point(FULL_FIELD - 8, 55);
    private static final Point RED_GOAL_ENTRANCE = new Point(8, 55);

    boolean isRed;
    PurePursuit wallridePursuit = null;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap, telemetry, this, false);
            robot = new Robot(hardwareMap, telemetry, this);
            isRed = robot.sensorThread.getPose().isInRange(0, 0, 70, 140);
            robot.sensorThread.resetPose(new Pose(6.5, 70, Math.PI / 2));
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
        double distanceToBlueEntrace = pose.distance(BLUE_GOAL_ENTRANCE);
        double distanceToRedEntrance = pose.distance(RED_GOAL_ENTRANCE);

        if (distanceToBlueEntrace < distanceToRedEntrance)
            return BLUE_GOAL_ENTRANCE;
        else
            return RED_GOAL_ENTRANCE;
    }

    private void startFollowingWall() {
        WayPoint start = new WayPoint(robot.sensorThread.getPose());

        // hack so purepursuit doesn't hit the wall when trying to pivot
        // TODO:  make pure pursuit do this for us
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
                24,
                new AngleLock(isInGoal() ? LOCK : NO_LOCK, entrancePose.heading)
        );

        WayPoint[] path = isInGoal() ?
                new WayPoint[]{start, pivot, insideGoal, entrance} :
                new WayPoint[]{start, pivot, entrance, insideGoal};

        wallridePursuit = new PurePursuit(robot, path, .2);
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

        if (inSlowMode()) {
            yMov /= 2;
            xMov /= 2;
            turnMov /= 2;
        }

        if (gamepad1.x && !isFollowingWall())
            startFollowingWall();

        if (isFollowingWall()) {
            wallridePursuit.update();
            if (wallridePursuit.atEnd() || !inputMovementsZero())
                stopFollowingWall();
        } else
            setDrivetrainMovements(yMov, xMov, turnMov);
    }

    private boolean inSlowMode() {
        return gamepad1.right_bumper;
    }

    private void setDrivetrainMovements(double yMov, double xMov, double turnMov) {
        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.requestRetraction();
        }
    }

    Button extendMode = new Button();

    private void updateOuttakeStates() {

        if (gamepad2.left_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_INWARDS);
        else if (gamepad2.right_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS);

        boolean dpad_up = extendMode.isToggled(gamepad2.dpad_up);
        if (gamepad2.dpad_up) {
            robot.outtakeModule.raise();

            if(robot.outtakeModule.getSlideLevel() == OuttakeModule.VerticalSlideLevel.TOP || robot.outtakeModule.getSlideLevel() == OuttakeModule.VerticalSlideLevel.TOP_TOP) {
//                maxExtend = !maxExtend;

                if (!dpad_up)
                    robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.TOP_TOP);
                else
                    robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.TOP);
            } else {
                extendMode = new Button();
                robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.TOP);
            }

        } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            robot.outtakeModule.raise();
            robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.MID);
        } else if (gamepad2.dpad_down) {
            robot.outtakeModule.raise();
            robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.DOWN);
        }
        if (gamepad2.x)
            robot.outtakeModule.pivot270();
        else if (gamepad2.y)
            robot.outtakeModule.pivotStraight();
        else if (gamepad2.b)
            robot.outtakeModule.pivotRight();
//        else if (gamepad2.b)
//            robot.outtakeModule.pivot270();

    }

    private void updateCarouselStates() {
        if (gamepad2.right_trigger > 0.01) {
            robot.carouselModule.clockwise = true;
        } else if (gamepad2.left_trigger > 0.01) {
            robot.carouselModule.clockwise = false;
        }
        robot.carouselModule.spin = gamepad2.left_trigger > 0.01 || gamepad2.right_trigger > 0.01;
    }
}
