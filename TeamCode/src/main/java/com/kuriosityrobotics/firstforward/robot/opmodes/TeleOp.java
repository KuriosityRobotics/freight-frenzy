package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import javassist.NotFoundException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot;

        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (NotFoundException e) {
            this.stop();
            return;
        }

        waitForStart();
        robot.start();

        while (opModeIsActive()) {
            // yeet
        }
    }
}
