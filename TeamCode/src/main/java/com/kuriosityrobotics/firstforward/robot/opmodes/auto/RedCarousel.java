package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedCarousel extends LinearOpMode {
    public static final Pose START = new Pose(6, 94.5, Math.toRadians(90));

    public static final Pose CAROUSEL = new Pose(12.8, 124, Math.toRadians(-75));

    public static final Pose WOBBLE = new Pose(32, 111, Math.toRadians(-27));

    public static final Pose WALL_ENT = new Pose(10, 100, Math.toRadians(180));
    public static final Pose WAREHOUSE_PARK = new Pose(31, 126, Math.toRadians(0));
    public static final Pose PARK = new Pose(8, 28, Math.toRadians(180));

    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        robot.resetPose(START);
        robot.carouselModule.clockwise = false;
        TeamMarkerDetector.startLocation = TeamMarkerDetector.AutoStartLocation.RED_DUCKS;

        waitForStart();

        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(robot.carouselModule.carouselAction());
        PurePursuit toCarousel = new PurePursuit(new WayPoint[]{
                new WayPoint(START),
                new WayPoint(START.x + 20, START.y + 7),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 10),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        ArrayList<Action> other = new ArrayList<>();
        other.add(robot.outtakeModule.extendOuttakeAction(robot.visionThread.getTeamMarkerDetector().getLocation().slideLevel()));
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        PurePursuit toWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL, other),
                new WayPoint(CAROUSEL.between(WOBBLE)),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

//        PurePursuit toPark = new PurePursuit(new ActionExecutor(hardwareMap), new WayPoint[]{
//                new WayPoint(CAROUSEL),
//                new WayPoint(WALL_ENT, 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(PARK, 0, new ArrayList<>())
//        }, 4);

        PurePursuit toPark = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(CAROUSEL.between(WAREHOUSE_PARK)),
                new WayPoint(WAREHOUSE_PARK, 0, new ArrayList<>())
        }, 4);

        // DETECT the THING

        // go to carousel
        robot.followPath(toCarousel);

        // to wobble
        robot.followPath(toWobble);

        robot.followPath(toPark);
    }
}