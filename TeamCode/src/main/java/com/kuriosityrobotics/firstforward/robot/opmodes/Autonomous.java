package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        PurePursuit pp = new PurePursuit(new WayPoint[]{
                new WayPoint(0, 0),
                new WayPoint(0, 20),
                new WayPoint(-30, 40),
                new WayPoint(-30, 60)
        }, 10);

        waitForStart();

        while (opModeIsActive()) {
            //pp.update(robot, 1);
            robot.drivetrain.setBrakePose(new Pose(10,10,Math.PI));
        }
    }
}
