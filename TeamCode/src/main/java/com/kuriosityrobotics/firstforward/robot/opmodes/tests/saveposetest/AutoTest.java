package com.kuriosityrobotics.firstforward.robot.opmodes.tests.saveposetest;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.PoseSaver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.io.IOException;

@Autonomous
public class AutoTest extends LinearOpMode {
    public static final Pose PARK = new Pose(8, 28, Math.toRadians(180));

    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            try {
                PoseSaver.savePose(PARK);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}

