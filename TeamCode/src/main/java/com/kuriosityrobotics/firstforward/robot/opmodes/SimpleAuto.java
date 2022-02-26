package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SimpleAuto extends LinearOpMode {
    private static final Pose START = new Pose((11.5 / 2), 71, Math.toRadians(180));

    private static final Pose PARK = RedAuto.PARK;

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

        PurePursuit park = new PurePursuit(new WayPoint[]{
                new WayPoint(START),
                new WayPoint(PARK, 0, new ArrayList<>())
        }, 5);

        waitForStart();

        // go to carousel
        robot.followPath(park);
//        long startTime = SystemClock.elapsedRealtime();
//        while (SystemClock.elapsedRealtime() < startTime + 1500) {
//            robot.drivetrain.setMovements(0, 0.5, 0);
//        }
    }
}
