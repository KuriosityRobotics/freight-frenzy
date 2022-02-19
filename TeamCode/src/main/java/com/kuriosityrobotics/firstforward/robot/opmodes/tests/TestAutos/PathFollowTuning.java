package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PauseAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PathFollowTuning extends LinearOpMode {
    public static final Pose START_W = new Pose(6.5, 64.5, Math.toRadians(90)); //start near warehouse
    public static final Pose START_C = new Pose(6.5, 108., Math.toRadians(90)); //start near carousel

    public static final Pose WOBBLE_W = new Pose(25.5, 70.5, Math.toRadians(240));
    public static final Pose WAREHOUSE = new Pose(6.25, 24., Math.toRadians(180));
    //public static final Pose WAREHOUSE_IN_RIDE = new Pose(4.5, 35, Math.toRadians(-180));
    //public static final Pose WAREHOUSE_OUT_RIDE = new Pose(6.5, 54.25, Math.toRadians(-180));

    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));
    public static final Pose WALL_GAP = new Pose(5.6, 47.25, Math.toRadians(180));
    public static final Pose BETWEEN_WOBBLE_WALLGAP = new Pose(7.5, 69. /*nice*/, Math.toRadians(210));
    public static final Pose STORAGE = new Pose(36., 128.5, Math.toRadians(-90));

    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
            robot.resetPose(START_W);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(robot.carouselModule.carouselAction());
        //carouselActions.add(new CarouselAction());
        PurePursuit startcToCarousel = new PurePursuit(new WayPoint[]{
                new WayPoint(START_C),
                new WayPoint(START_C.x + 20, START_C.y + 4, new VelocityLock(0.5 * MotionProfile.ROBOT_MAX_VEL)),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 3);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        wobbleActions.add(new PauseAction());
        //wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(robot.intakeModule.intakeAction());
        wobbleActions.add(new PauseAction());
        //intakeActions.add(new IntakeAction());
        PurePursuit wobbleToWarehouse = new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE_W),
                new WayPoint(BETWEEN_WOBBLE_WALLGAP, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WALL_GAP, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WAREHOUSE, 0, intakeActions)
        }, 4);

        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(WAREHOUSE),
                new WayPoint(WALL_GAP, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(BETWEEN_WOBBLE_WALLGAP, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, wobbleActions)
        }, 4);

        PurePursuit startwToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(START_W),
                new WayPoint(START_W.between(WOBBLE_W), 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, wobbleActions)
        }, 4);

        PurePursuit carouselToStorage = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(CAROUSEL.between(STORAGE), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(STORAGE, 0, new ArrayList<>())
        }, 5);

        waitForStart();

        robot.followPath(startwToWobble);

        robot.followPath(wobbleToWarehouse);

        robot.followPath(warehouseToWobble);
    }
}

