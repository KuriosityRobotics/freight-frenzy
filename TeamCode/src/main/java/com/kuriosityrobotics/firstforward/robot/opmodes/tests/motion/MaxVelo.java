package com.kuriosityrobotics.firstforward.robot.opmodes.tests.motion;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.opmodes.auto.RedCarousel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxVelo extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        robot.resetPose(RedCarousel.START);

        waitForStart();

        boolean flip = false;
        while (opModeIsActive()) {
            if (!flip) {
                robot.drivetrain.setMovements(0.5, 0.5, 0);

                if (robot.distanceToPoint(new Point(0, 0)) > 60) {
                    flip = true;
                }
            } else {
                robot.drivetrain.setMovements(-0.5, -0.5, 0);
            }
        }
    }
}
