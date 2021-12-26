package com.kuriosityrobotics.firstforward.robot.opmodes.tests.saveposetest;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.PoseSaver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.IOException;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose pose = null;

        try {
            pose = PoseSaver.readPose();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();

        while (opModeIsActive()) {
            // yeet
            assert pose != null; // idk how this got here but it's irrelevant :sunglas:

            telemetry.addData("Pose: ", "hella " + pose.toString());
            telemetry.addData("Save Method: ", "hella " + PoseSaver.getSaveMethod());
            telemetry.update();
        }
    }
}
