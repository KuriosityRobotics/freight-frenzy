package com.kuriosityrobotics.firstforward.robot.opmodes.tests.motion;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.opmodes.auto.RedCarousel;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxVelo extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        robot.resetPose(new Pose(0, 3, 0));

        waitForStart();

        boolean flip = false;
        while (opModeIsActive()) {
            if (robot.distanceToPoint(new Point(0, 0)) > 20 || robot.distanceToPoint(Pose.ZERO) < 1) {
                flip = !flip;
            }

            if (!flip) {
                robot.drivetrain.setMovements(0.5, 0.5, 0);
            } else {
                robot.drivetrain.setMovements(-0.5, -0.5, 0);
            }
        }
    }
}
