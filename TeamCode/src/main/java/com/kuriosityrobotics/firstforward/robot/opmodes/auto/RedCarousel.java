package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedCarousel extends LinearOpMode {
    public static final Pose START = new Pose(9.5, (23.5*5)-0.5, Math.toRadians(-90));

    public static final Pose WOBBLE = new Pose(32, 111, Math.toRadians(-27));

    public static final Point PRE_CAROUSEL = new Point(18, 118);
    public static final Pose CAROUSEL = new Pose(12.8, 124, Math.toRadians(-75));

    public static final Pose PARK = new Pose(36, 5*23.5 + 12, Math.toRadians(-90));

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
        Robot.isCarousel = true;

        waitForStart();

        OuttakeModule.VerticalSlideLevel detected = robot.visionThread.getTeamMarkerDetector().getLocation().slideLevel();

        PurePursuit toWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(START, robot.outtakeModule.extendOuttakeAction(detected)),
                new WayPoint(WOBBLE, 0, robot.outtakeModule.dumpOuttakeAction())
        }, true, 4);

        PurePursuit toCarousel = new PurePursuit(new WayPoint[]{
                new WayPoint(WOBBLE),
                new WayPoint(PRE_CAROUSEL, 10),
                new WayPoint(CAROUSEL, 0, robot.carouselModule.carouselAction())
        }, false, 4);

        PurePursuit toPark = new PurePursuit(new WayPoint[]{
                new WayPoint(CAROUSEL),
                new WayPoint(PARK)
        }, true, 4);

        robot.followPath(toWobble);
        robot.followPath(toCarousel);
        robot.followPath(toPark);

        Robot.isCarousel = false;
    }
}