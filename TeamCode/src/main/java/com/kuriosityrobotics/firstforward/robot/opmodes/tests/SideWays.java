package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SideWays extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            e.printStackTrace();
        }

        PurePursuit pp = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(0, 0, new AngleLock(0)),
                new WayPoint(20, 0),
                new WayPoint(50, 0, 0)
        }, 10);

        waitForStart();

        pp.follow();

        while (opModeIsActive()) {
            // yeet
        }
    }
}
