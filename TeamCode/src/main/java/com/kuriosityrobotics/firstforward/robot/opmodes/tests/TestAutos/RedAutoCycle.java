package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Objects;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedAutoCycle extends LinearOpMode {
    public enum RedAutoCycleState {
        START,
        DEPOSIT,
        CYCLE,
        PARK
    }

    public RedAutoCycleState state = RedAutoCycleState.START;

    public static final Pose START = new Pose(6, 94.5, Math.toRadians(90));

    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));

    public static final Pose WOBBLE = new Pose(36, 93, Math.toRadians(-30));
    public static final Pose WAREHOUSE = new Pose(8, 15, Math.toRadians(180));
    public static final Pose WALL_WH = new Pose(5, 93, Math.toRadians(90));

    public static final Pose WALL_ENT = new Pose(9, 100, Math.toRadians(180));
    public static final Pose PARK = new Pose(8, 28, Math.toRadians(180));

    public void runOpMode() {
        Robot robot = null;

        try {
            robot = new Robot(hardwareMap, telemetry, this);
            robot.resetPose(START);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(robot.carouselModule.carouselAction());
        PurePursuit toCarousel = new PurePursuit(new WayPoint[]{
                new WayPoint(START),
                new WayPoint(START.x + 20, START.y + 4, new VelocityLock(0.5 * MotionProfile.ROBOT_MAX_VEL)),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        PurePursuit carouselToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL, robot.outtakeModule.extendOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP)),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(robot.intakeModule.intakeAction());
        PurePursuit toWarehouse = new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(WALL_WH,  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WAREHOUSE, 0, intakeActions)
        }, 4);

        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(WAREHOUSE),
                new WayPoint(WALL_WH,  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

        PurePursuit toPark = new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(WALL_WH, 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(PARK, 0, new ArrayList<>())
        }, 4);

        PurePursuit toWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(START),
                new WayPoint(WOBBLE, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
        }, 4);

        waitForStart();

        switch (state) {
            case START:
                robot.followPath(toWobble);
                state = RedAutoCycleState.DEPOSIT;
            case DEPOSIT:
                switch (Objects.requireNonNull(robot).visionThread.getTeamMarkerDetector().getLocation()) { // please java shut the GELL up
                    case LEVEL_1:
                        // level 1 action
                        robot.followPath(createOutTakeAction(OuttakeModule.VerticalSlideLevel.DOWN, robot));
                        state = RedAutoCycleState.CYCLE;
                        break;
                    case LEVEL_2:
                        // level 2 action
                        robot.followPath(createOutTakeAction(OuttakeModule.VerticalSlideLevel.MID, robot));
                        state = RedAutoCycleState.CYCLE;
                        break;
                    case LEVEL_3:
                        // level 3 action
                        robot.followPath(createOutTakeAction(OuttakeModule.VerticalSlideLevel.TOP, robot));
                        state = RedAutoCycleState.CYCLE;
                        break;
                }
            case CYCLE:
                for (int i = 0; i < 4; i++) {
                    robot.followPath(toWarehouse);
                    robot.followPath(toWobble);
                }
                state = RedAutoCycleState.PARK;
            case PARK:
                robot.followPath(toPark);
                break;
        }
    }

    public PurePursuit createOutTakeAction(OuttakeModule.VerticalSlideLevel level, Robot robot) {
        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.extendOuttakeAction(level));
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        return new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE, 0, wobbleActions),
        }, 4);
    }
}
