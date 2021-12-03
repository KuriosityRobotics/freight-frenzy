package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.CarouselAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.DumpOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.RaiseOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private static final Pose START = new Pose(6, 94.5, Math.toRadians(90));
    private static final Pose CAROUSEL = new Pose(12.5, 122, Math.toRadians(-80));
    private static final Pose WOBBLE = new Pose(36, 93, Math.toRadians(-30));

    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this, START);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        ArrayList<Action> carouselActions = new ArrayList<>();
        carouselActions.add(new CarouselAction());
        PurePursuit toCarousel = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(START),
                new WayPoint(START.x + 20, START.y + 4, 0.5 * MotionProfile.ROBOT_MAX_VEL),
                new WayPoint(CAROUSEL.x, CAROUSEL.y - 5, CAROUSEL.heading, 3),
                new WayPoint(CAROUSEL.x, CAROUSEL.y, CAROUSEL.heading, 0, carouselActions)
        }, 4);

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(new DumpOuttakeAction(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS));
        PurePursuit toWobble = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(CAROUSEL, new RaiseOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP)),
                new WayPoint((CAROUSEL.x + WOBBLE.x) / 2., (CAROUSEL.y + WOBBLE.y) / 2.),
                new WayPoint(WOBBLE, 0, wobbleActions)
        }, 4);

        waitForStart();

        // DETECT the THING

        // go to carousel
        toCarousel.follow();

        // to wobble
        toWobble.follow();
    }
}
