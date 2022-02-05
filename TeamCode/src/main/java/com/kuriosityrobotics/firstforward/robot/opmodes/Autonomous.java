package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.CarouselAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.DumpOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.IntakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.RaiseOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.sensors.SensorThread;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.dataflow.qual.Pure;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {

    public static final Pose START = new Pose(6, 94.5, Math.toRadians(90));

    public static final Pose START_W = new Pose(6, 60, Math.toRadians(90)); //start near warehouse
    public static final Pose START_C = new Pose(6, 108, Math.toRadians(90)); //start near carousel

    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));
    public static final Pose WOBBLE_W = new Pose(25.5, 72, Math.toRadians(-110));
    public static final Pose WOBBLE_C = new Pose(36, 93, Math.toRadians(-30));
    public static final Pose WAREHOUSE = new Pose(8, 15, Math.toRadians(180));
    public static final Pose BETWEEN_START_WOBBLE = new Pose(21, 65.25, Math.toRadians(-110));
    public static final Pose WALL_GAP = new Pose(6.5, 47.25, 180);
    public static final Pose BETWEEN_WOBBLE_WALLGAP = new Pose(12, 72, Math.toRadians(-110));
    public static final Pose STORAGE = new Pose(36, 132, Math.toRadians(-90));

    public static PurePursuit startcToCarousel;
    public static PurePursuit wobbleToWarehouse;
    public static PurePursuit warehouseToWobble;
    public static PurePursuit startwToWobble;
    public static PurePursuit carouselToStorage;
    public static PurePursuit startcToWobble;
    /*
    public static final Pose CAROUSEL = new Pose(12.5, 125.5, Math.toRadians(-80));

    public static final Pose WOBBLE = new Pose(36, 93, Math.toRadians(-30));
    public static final Pose WAREHOUSE = new Pose(8, 15, Math.toRadians(180));
    public static final Pose WALL_WH = new Pose(5, 93, Math.toRadians(90));

    public static final Pose WALL_ENT = new Pose(9, 100, Math.toRadians(180));

     */
    public static final Pose PARK = new Pose(8, 28, Math.toRadians(180));


    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        robot.sensorThread.resetPose(START);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(new IntakeAction());

        /*
        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(new CarouselAction());
        PurePursuit toCarousel = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START),
                new WayPoint(START.x + 20, START.y + 4, 0.5 * MotionProfile.ROBOT_MAX_VEL),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));
        PurePursuit carouselToWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL, new RaiseOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP)),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(new IntakeAction());
        PurePursuit toWarehouse = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(WALL_WH,  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WAREHOUSE, 0, intakeActions)
        }, 4);

        PurePursuit warehouseToWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WAREHOUSE),
                new WayPoint(WALL_WH,  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

        PurePursuit toPark = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(WALL_WH, 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(PARK, 0, new ArrayList<>())
        }, 4);
         */

        waitForStart();

        // DETECT the THING

        // go to carousel
        //toCarousel.follow();

        // go from carousel to wobble
        //carouselToWobble.follow();

        //place 3 more freight on wobble
        //for (int i = 0; i < 3; i++){
        //    toWarehouse.follow();
        //    warehouseToWobble.follow();
        //}

        //toPark.follow();
    }

    public static void initPurePursuit(Robot robot) {
        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(new CarouselAction());

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));

        ArrayList<Action> intakeActions = new ArrayList<>();
        intakeActions.add(new IntakeAction());

        Autonomous.startcToCarousel = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START_C),
                new WayPoint(START_C.x + 20, START_C.y + 4, 0.5 * MotionProfile.ROBOT_MAX_VEL),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        Autonomous.wobbleToWarehouse = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WOBBLE_W),
                new WayPoint(new Pose(BETWEEN_WOBBLE_WALLGAP, Math.toRadians(-150)),  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(new Pose(WALL_GAP,  180), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WAREHOUSE, 0, intakeActions)
        }, 4);

        Autonomous.startwToWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START_W),
                new WayPoint(BETWEEN_START_WOBBLE , 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, wobbleActions)
        }, 4);

        Autonomous.carouselToStorage = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(CAROUSEL.between(STORAGE), 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(STORAGE, 0, new ArrayList<>())
        }, 4);

        Autonomous.warehouseToWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(WAREHOUSE),
                new WayPoint(WALL_GAP, 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(BETWEEN_WOBBLE_WALLGAP,  0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_W, 0, wobbleActions)
        }, 4);

        Autonomous.startcToWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START_C),
                new WayPoint(START_C.between(WOBBLE_C), 0.5 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE_C, 0, wobbleActions)
        }, 4);
    }
}
