package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    @Override
    public void runOpMode(){
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }
        waitForStart();
        FileDump.activate();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
        }
    }

    private void updateDrivetrainStates() {
        double yMov = -gamepad1.left_stick_y;
        double xMov = gamepad1.left_stick_x;
        double turnMov = gamepad1.right_stick_x;

        //Log.i("TeleOp", "brake: " + robot.drivetrain.brake);
        //Log.i("TeleOp", "brake point: " + robot.drivetrain.brakePoint.toString());
        //Log.i("TeleOp", "current pos: " + robot.sensorThread.odometry.getPosition().toString());
        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }
}
