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
            throw new RuntimeException(e);
        }

        waitForStart();
        robot.start();

        while (opModeIsActive()) {
//            if (gamepad1.a)
//                robot.blockerModule.setCurrentState(robot.blockerModule.OpenFlap());
//            else if (gamepad1.b)
//                robot.blockerModule.setCurrentState(robot.blockerModule.CloseFlap());

            // yeet
        }
    }
}
