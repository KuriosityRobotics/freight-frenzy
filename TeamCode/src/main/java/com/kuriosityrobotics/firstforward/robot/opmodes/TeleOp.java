package com.kuriosityrobotics.firstforward.robot.opmodes;

import android.os.SystemClock;

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

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
            updateIntakeStates();
        }
    }

    private final double EPSILON = 0.1;
    private void updateDrivetrainStates() {
        double yMov = -gamepad1.left_stick_y;
        double xMov = gamepad1.left_stick_x;
        double turnMov = gamepad1.right_stick_x;
        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates(){
        double intakeMove = Math.abs(gamepad2.left_stick_y)
                > EPSILON ? Math.signum(gamepad2.left_stick_y) : 0;
        robot.intakeModule.setPower(-intakeMove);

        if(gamepad2.a)
            robot.intakeModule.startIntakeRetraction();
        //right bumper to un-intake in case stuff gets stuck or something
//        robot.intakeModule.intakePow = toNum(gamepad1.left_bumper) - toNum(gamepad1.right_bumper);
//        robot.intakeModule.doorOpen = gamepad1.a;
//        robot.intakeModule.extenderPos = gamepad1.right_trigger;
    }
}
