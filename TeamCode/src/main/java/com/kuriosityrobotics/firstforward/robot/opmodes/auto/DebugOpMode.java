package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DebugOpMode extends LinearOpMode {
    public void runOpMode() {
        var originalDebug = Robot.DEBUG;
        Robot.DEBUG = true;

        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }
        robot.visionThread.vuforiaLocalizationConsumer.manualCam = true;
        robot.visionThread.vuforiaLocalizationConsumer.manualAngle = PI;

        waitForStart();
        while(!isStopRequested());
        Robot.DEBUG = originalDebug;
    }
}
