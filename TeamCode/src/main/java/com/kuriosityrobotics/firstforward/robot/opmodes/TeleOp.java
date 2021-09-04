package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import javassist.NotFoundException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (NotFoundException e) {
            this.stop();
            throw new RuntimeException(e);
        }

        waitForStart();
        robot.start();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
        }
    }

    private void updateDrivetrainStates(){
        double yMov = -gamepad1.left_stick_y;
        double xMov = gamepad1.left_stick_x;
        double turnMov = gamepad1.right_stick_x;

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }
}
