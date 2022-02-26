package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PathFollowTuning extends LinearOpMode {
//    public static final Pose START_C = new Pose(6.5, 108., Math.toRadians(90)); //start near carousel
//
//    public static final Pose RED_START_W = new Pose(7, 64.5, Math.toRadians(90)); //start near warehouse
//    public static final Pose RED_WOBBLE_W = new Pose(22.5, 71.5, Math.toRadians(250));
//    public static final Point RED_BETWEEN_WOBBLE_WALLGAP = new Point(7, 61.5);
//    public static final Pose RED_WALL_GAP = new Pose(6, 46.5, Math.toRadians(180));
//    public static final Pose RED_WAREHOUSE = new Pose(6.2, 23, Math.toRadians(180));
//
//    public static final Pose BLUE_START_W = new Pose(140.5-6.75, 64.5, Math.toRadians(-90)); //start near warehouse
//    public static final Pose BLUE_WOBBLE_W = new Pose(140.5-25.5, 70.5, Math.toRadians(-240));
//    public static final Pose BLUE_BETWEEN_WOBBLE_WALLGAP = new Pose(10, 66., Math.toRadians(-200));
//    public static final Pose BLUE_WALL_GAP = new Pose(140.5-5.8, 46.5, Math.toRadians(-180));
//    public static final Pose BLUE_WAREHOUSE = new Pose(140.5-5.85, 20, Math.toRadians(-180));
//    //public static final Pose WAREHOUSE_IN_RIDE = new Pose(4.5, 35, Math.toRadians(-180));
//    //public static final Pose WAREHOUSE_OUT_RIDE = new Pose(6.5, 54.25, Math.toRadians(-180));
//
//    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));
//    public static final Pose STORAGE = new Pose(36, 128.5, Math.toRadians(-90));
//
    public void runOpMode() {
//        Robot robot = null;
//        try {
//            robot = new Robot(hardwareMap, telemetry, this);
//            robot.resetPose(RED_START_W);
//        } catch (Exception e) {
//            this.stop();
//            e.printStackTrace();
//            return;
//        }
//
//        ArrayList<Action> carouselActions = new ArrayList<>();
//        //carouselActions.add(new CarouselAction());
//        PurePursuit startcToCarousel = new PurePursuit(new WayPoint[]{
//                new WayPoint(START_C),
//                new WayPoint(START_C.x + 20, START_C.y + 4, new VelocityLock(0.5 * MotionProfile.ROBOT_MAX_VEL)),
//                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
//                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
//        }, 3, "start to carousel");
//
//        ArrayList<Action> wobbleActions = new ArrayList<>();
//        //wobbleActions.add(new PauseAction());
//        //wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));
//
//        ArrayList<Action> intakeActions = new ArrayList<>();
//        //intakeActions.add(new PauseAction());
//        //intakeActions.add(new IntakeAction());
//        PurePursuit redStartwToWobble = new PurePursuit(new WayPoint[]{
//                new WayPoint(RED_START_W),
//                //new WayPoint(RED_START_W.between(RED_WOBBLE_W) , 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(RED_WOBBLE_W, 0, wobbleActions)
//        }, 4, "start to wobble");
//
//        PurePursuit wobbleToWarehouse = new PurePursuit(new WayPoint[]{
//                new WayPoint(RED_WOBBLE_W),
//                new WayPoint(RED_BETWEEN_WOBBLE_WALLGAP),//, 0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(RED_WALL_GAP),//, 0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(RED_WAREHOUSE, 0, intakeActions)
//        }, 4, "wobble to warehouse");
//
//        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
//                new WayPoint(RED_WAREHOUSE),
//                new WayPoint(RED_WALL_GAP),//,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(RED_BETWEEN_WOBBLE_WALLGAP),//,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(RED_WOBBLE_W, 0, wobbleActions)
//        }, 4, "warehouse to wobble");
//
//
//        PurePursuit blueStartwToWobble = new PurePursuit(new WayPoint[]{
//                new WayPoint(BLUE_START_W),
//                //new WayPoint(BLUE_START_W.between(BLUE_WOBBLE_W) , 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(BLUE_WOBBLE_W, 0, wobbleActions)
//        }, 4, "start to wobble");
//
//        PurePursuit blueWobbleToWarehouse = new PurePursuit(new WayPoint[]{
//                new WayPoint(BLUE_WOBBLE_W),
//                new WayPoint(BLUE_BETWEEN_WOBBLE_WALLGAP),//, 0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(BLUE_WALL_GAP),//, 0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(BLUE_WAREHOUSE, 0, intakeActions)
//        }, 4, "wobble to warehouse");
//
//        PurePursuit blueWarehouseToWobble = new PurePursuit(new WayPoint[]{
//                new WayPoint(BLUE_WAREHOUSE),
//                new WayPoint(BLUE_WALL_GAP,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(BLUE_BETWEEN_WOBBLE_WALLGAP,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(BLUE_WOBBLE_W, 0, wobbleActions)
//        }, 4, "warehouse to wobble");
//
//        PurePursuit carouselToStorage  = new PurePursuit(new WayPoint[]{
//                new WayPoint(CAROUSEL),
//                new WayPoint(CAROUSEL.between(STORAGE), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(STORAGE, 0, new ArrayList<>())
//        }, 5, "carousel to storage");
//
//        waitForStart();
//
//        robot.followPath(redStartwToWobble);
//
//        for (int  i = 0; i < 3; i++){
//            robot.followPath(wobbleToWarehouse);
//            robot.followPath(warehouseToWobble);
        }
//
//
//
//        /*
//        blueStartwToWobble.follow(false);
//
//        for (int i = 0; i < 3; i++){
//            robot.followPath(blueWobbleToWarehouse);
//            robot.followPath(blueWarehouseToWobble);
//        }
//         */
//
//    }
}
