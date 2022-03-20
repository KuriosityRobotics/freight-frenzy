package com.kuriosityrobotics.firstforward.robot.opmodes.tests.motion;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Straight extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        robot.resetPose(new Pose(0, 0, 0));

        PurePursuit pp = new PurePursuit(new WayPoint[]{
                new WayPoint(0, 0),
                new WayPoint(0, 20),
                new WayPoint(0, 40),
//                new WayPoint(-30, 60, 0)
                new WayPoint(0, 50, new VelocityLock(0))
        }, 10);

        waitForStart();

        robot.followPath(pp);

        while (opModeIsActive()) {
            // yeet
        }
    }
}
