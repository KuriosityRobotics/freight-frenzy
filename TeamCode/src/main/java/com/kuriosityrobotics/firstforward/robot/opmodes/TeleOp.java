package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    @Override
    public void runOpMode(){
        try {
            // TODO: change
            robot = new Robot(hardwareMap, telemetry, this, new Pose(6.5, 75.5, Math.toRadians(90)));
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }
        waitForStart();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();

            if (gamepad1.a) {
                robot.getVisionThread().getManagedCamera().activateCamera(robot.cameraName1);
            } else if (gamepad1.b) {
                robot.getVisionThread().getManagedCamera().activateCamera(robot.cameraName2);
            }
        }
    }

    private void updateDrivetrainStates() {
        double yMov = -gamepad1.left_stick_y;
        double xMov = gamepad1.left_stick_x;
        double turnMov = gamepad1.right_stick_x;

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }
}
