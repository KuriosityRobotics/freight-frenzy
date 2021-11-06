package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.kuriosityrobotics.firstforward.robot.vision.WebcamLocalization;

@TeleOp
public class VisionTest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }

        robot.start();

//        WebcamLocalization vuforiaLocalizer = new WebcamLocalization(robot);

        waitForStart();

        while (opModeIsActive()) {
            sleep(500);
        }

//        vuforiaLocalizer.stopConsuming();
    }
}