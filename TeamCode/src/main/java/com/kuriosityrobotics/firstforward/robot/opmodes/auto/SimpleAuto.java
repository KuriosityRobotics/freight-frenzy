package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SimpleAuto extends LinearOpMode {
    private static final Pose START = Pose.fieldMirror(9.75, 64.5, Math.toRadians(90));

    private static final Pose PARK = BlueCycle.blueWarehouse;

    public void runOpMode() {
        Robot robot;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        Robot.isBlue = true;
        robot.resetPose(Pose.fieldMirror(9.75, (23.5 * 5) - 0.5 - (11.5 / 2), Math.toRadians(-90)));
            AutoPaths.calibrateVuforia(robot);

        PurePursuit park = new PurePursuit(new WayPoint[]{
                new WayPoint(START),
                new WayPoint(PARK.x, PARK.y, new AngleLock(PARK.heading))
        }, 5);

        PurePursuit toStart = new PurePursuit(new WayPoint[]{
                new WayPoint(PARK.x, PARK.y, new AngleLock(PARK.heading)),
                new WayPoint(START)
        }, 5);

        waitForStart();
        robot.resetPose(START);

        // go to carousel
        while (opModeIsActive() && !isStopRequested()) {
            robot.followPath(park);
            robot.followPath(toStart);
        }
//        long startTime = SystemClock.elapsedRealtime();
//        while (SystemClock.elapsedRealtime() < startTime + 1500) {
//            robot.drivetrain.setMovements(0, 0.5, 0);
//        }
    }
}
