package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxVelo extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            e.printStackTrace();
        }

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
