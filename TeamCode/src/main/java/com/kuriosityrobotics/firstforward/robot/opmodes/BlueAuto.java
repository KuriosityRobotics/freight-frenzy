package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.CarouselAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.DumpOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.ExtendOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BlueAuto extends LinearOpMode {
    public static final Pose START = new Pose(140.5-6, 94.5, Math.toRadians(-90));

    public static final Pose CAROUSEL = new Pose(140.5-12.8-3.25, 124, Math.toRadians(68));

    public static final Pose WOBBLE = new Pose(140.5-31.6, 107.75, Math.toRadians(25));

    public static final Pose WALL_ENT = new Pose(10, 100, Math.toRadians(180));
    public static final Pose WAREHOUSE_PARK = new Pose(140.5-32, 126, Math.toRadians(0));
    public static final Pose PARK = new Pose(8, 28, Math.toRadians(180));

    public void runOpMode() {

        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            stop();
            e.printStackTrace();
            return;
        }

        Robot.isBlue = true;

        robot.sensorThread.resetPose(START);
        robot.carouselModule.clockwise = true;
        TeamMarkerDetector.startLocation = TeamMarkerDetector.AutoStartLocation.BLUE_DUCKS;

        waitForStart();

        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(new CarouselAction());
        PurePursuit toCarousel = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START),
                new WayPoint(START.x - 20, START.y + 4, new VelocityLock(0.5 * MotionProfile.ROBOT_MAX_VEL)),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 7.5, CAROUSEL.heading, 4),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        ArrayList<Action> other = new ArrayList<>();
        other.add(new ExtendOuttakeAction(robot.visionThread.teamMarkerDetector.getLocation().slideLevel()));
        wobbleActions.add(new DumpOuttakeAction());
        PurePursuit toWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL, other),
                new WayPoint(CAROUSEL.between(WOBBLE), 0.3 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

//        PurePursuit toPark = new PurePursuit(robot, new WayPoint[]{
//                new WayPoint(CAROUSEL),
//                new WayPoint(WALL_ENT, 0.2 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
//                new WayPoint(PARK, 0, new ArrayList<>())
//        }, 4);

        PurePursuit toPark = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(CAROUSEL.between(WAREHOUSE_PARK), 9, new ArrayList<>()),
                new WayPoint(WAREHOUSE_PARK, 0, new ArrayList<>())
        }, 4);

        // DETECT the THING

        // go to carousel
        toCarousel.follow(false);

        // to wobble
//        toWobble.follow(false);
//
//        toPark.follow(false);
    }
}