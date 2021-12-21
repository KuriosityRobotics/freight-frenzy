package com.kuriosityrobotics.firstforward.robot.opmodes.tests.saveposetest;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.CarouselAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.DumpOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.RaiseOuttakeAction;
import com.kuriosityrobotics.firstforward.robot.pathfollow.motionprofiling.MotionProfile;
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
                PoseSaver.removePose();
                PoseSaver.savePose(PARK);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}

