package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PathFollowTuning extends LinearOpMode {
    public static final Pose START_W = new Pose(6, 58.5, Math.toRadians(90)); //start near warehouse
    public static final Pose START_C = new Pose(6, 108, Math.toRadians(90)); //start near carousel

    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));
    public static final Pose WOBBLE_W = new Pose(25.5, 70.5, Math.toRadians(-110));
    public static final Pose WAREHOUSE = new Pose(6.5, 15, Math.toRadians(-180));
    public static final Pose BETWEEN_START_WOBBLE = new Pose(21, 65.25, Math.toRadians(-110));
    public static final Pose WALL_GAP = new Pose(6.5, 47.25, -180);
    public static final Pose BETWEEN_WOBBLE_WALLGAP = new Pose(12, 70.5, Math.toRadians(-110));
    public static final Pose STORAGE = new Pose(36, 128.5, Math.toRadians(-90));

    //public static final Pose WALL_ENT = new Pose(9, 100, Math.toRadians(180));

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
        PurePursuit startcToCarousel = new PurePursuit(new WayPoint[]{
                new WayPoint(START_C),
                new WayPoint(START_C.x + 20, START_C.y + 4, new VelocityLock(0.5 * MotionProfile.ROBOT_MAX_VEL)),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(robot.intakeModule.intakeAction());
        PurePursuit wobbleToWarehouse = new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE_W),
                new WayPoint(new Pose(BETWEEN_WOBBLE_WALLGAP, Math.toRadians(-150)), 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(new Pose(WALL_GAP, 180), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WAREHOUSE, 0, intakeActions)
        }, 4);

        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(WAREHOUSE),
                new WayPoint(WALL_GAP, 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(BETWEEN_WOBBLE_WALLGAP, 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, wobbleActions)
        }, 4);

        PurePursuit startwToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(START_W),
                new WayPoint(START_W.between(WOBBLE_W) , 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, new ArrayList<>())
        }, 4);

        PurePursuit carouselToStorage = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(CAROUSEL.between(STORAGE), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(STORAGE, 0, new ArrayList<>())
        }, 4);

        waitForStart();

        robot.followPath(startwToWobble);

        wobbleToWarehouse.follow(false);

        warehouseToWobble.follow(false);
    }
}
