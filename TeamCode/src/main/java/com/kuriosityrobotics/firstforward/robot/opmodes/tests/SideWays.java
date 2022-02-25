package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.ActionExecutor;
import com.kuriosityrobotics.firstforward.robot.pathfollow.AngleLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SideWays extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        robot.resetPose(new Pose(0, 0, 0));

        PurePursuit pp = new PurePursuit(new ActionExecutor(), new WayPoint[]{
                new WayPoint(0, 0, new AngleLock(0)),
                new WayPoint(-20, 0),
                new WayPoint(-50, 0, new VelocityLock(0))
        }, 10);

        waitForStart();

        robot.followPath(pp);

        while (opModeIsActive()) {
            // yeet
        }
    }
}
