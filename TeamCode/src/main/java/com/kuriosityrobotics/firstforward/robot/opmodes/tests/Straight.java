package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Straight extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        robot.sensorThread.resetPose(new Pose(0, 0, 0));

        PurePursuit pp = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(0, 0),
                new WayPoint(0, 20),
                new WayPoint(0, 40),
//                new WayPoint(-30, 60, 0)
                new WayPoint(0, 50, new VelocityLock(0))
        }, 10);

        waitForStart();

        pp.follow(false);

        while (opModeIsActive()) {
            // yeet
        }
    }
}
