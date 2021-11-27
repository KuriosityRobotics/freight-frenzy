package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxVelo extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this, new Pose(0, 0, 0));

        waitForStart();

        boolean flip = false;
        while (opModeIsActive()) {
            if (!flip) {
                robot.drivetrain.setMovements(0.5, 0.5, 0);

                if (robot.drivetrain.distanceToPoint(new Point(0, 0)) > 60) {
                    flip = true;
                }
            } else {
                robot.drivetrain.setMovements(-0.5, -0.5, 0);
            }
        }
    }
}
