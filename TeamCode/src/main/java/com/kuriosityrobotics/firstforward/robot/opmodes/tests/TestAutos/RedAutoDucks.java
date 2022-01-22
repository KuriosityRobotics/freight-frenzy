package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.DumpOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.RaiseOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedAutoDucks extends LinearOpMode {
    public enum RedAutoDucksState {
        START,
        DEPOSIT,
        CAROUSEL,
        PARK
    }

    public RedAutoDucksState state = RedAutoDucksState.START;

    public static final Pose START = new Pose(6, 94.5, Math.toRadians(90));
    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));
    public static final Pose WOBBLE = new Pose(36, 93, Math.toRadians(-30));
    public static final Pose PARK = new Pose(36, 126.5, Math.toRadians(180));

    public void runOpMode() {
        Robot robot = null;

        try {
            robot = new Robot(hardwareMap, telemetry, this, START);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        PurePursuit toWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START),
                new WayPoint(WOBBLE, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
        }, 4);

        PurePursuit wobbleToCarousel = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(CAROUSEL)
        }, 4);

        PurePursuit toPark = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(PARK, 0, new ArrayList<>())
        }, 4);

        waitForStart();

        switch (state) {
            case START:
                toWobble.follow();
                state = RedAutoDucksState.DEPOSIT;
            case DEPOSIT:
                switch (Objects.requireNonNull(robot).visionThread.teamMarkerDetector.getLocation()) { // please java shut the GELL up
                    case LEVEL_1:
                        // level 1 action
                        createOutTakeAction(OuttakeModule.VerticalSlideLevel.DOWN, robot).follow();
                        state = RedAutoDucksState.CAROUSEL;
                        break;
                    case LEVEL_2:
                        // level 2 action
                        createOutTakeAction(OuttakeModule.VerticalSlideLevel.MID, robot).follow();
                        state = RedAutoDucksState.CAROUSEL;
                        break;
                    case LEVEL_3:
                        // level 3 action
                        createOutTakeAction(OuttakeModule.VerticalSlideLevel.TOP, robot).follow();
                        state = RedAutoDucksState.CAROUSEL;
                        break;
                }
            case CAROUSEL:
                wobbleToCarousel.follow();
                state = RedAutoDucksState.PARK;
            case PARK:
                toPark.follow();
                break;
        }
    }

    public PurePursuit createOutTakeAction(OuttakeModule.VerticalSlideLevel level, Robot robot) {
        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));
        return new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WOBBLE, new RaiseOuttakeAction(level)),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);
    }
}
