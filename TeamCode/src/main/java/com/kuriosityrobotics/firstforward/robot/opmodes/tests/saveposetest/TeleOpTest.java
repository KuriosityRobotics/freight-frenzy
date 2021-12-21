package com.kuriosityrobotics.firstforward.robot.opmodes.tests.saveposetest;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.PoseSaver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose pose = new Pose(0,0,0);

        try {
            pose = PoseSaver.readPose();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pose: ", "hella " + pose.toString());
            telemetry.update();
            // yeet
        }
    }
}
