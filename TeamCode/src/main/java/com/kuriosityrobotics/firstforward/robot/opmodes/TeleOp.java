package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Toggle linkageToggle = new Toggle();
    Toggle hopperPivotToggle = new Toggle();

    private boolean prevY = false;

    @Override
    public void runOpMode() {
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
            //robot.drivetrain.setBrakePose(new Pose(0,24,0));
            updateIntakeStates();
            updateOuttakeStates();
            //robot.drivetrain.setBrakePose(new Pose(10,0,0));
        }
    }

    private final double EPSILON = 0.1;

    private void updateDrivetrainStates() {
        double yMov = -gamepad1.left_stick_y;
        double xMov = gamepad1.left_stick_x;
        double turnMov = gamepad1.right_stick_x;

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        double intakeMove = Math.abs(gamepad2.left_stick_y)
                > EPSILON ? Math.signum(gamepad2.left_stick_y) : 0;
        robot.intakeModule.setPower(intakeMove);

        if (gamepad2.a)
            robot.intakeModule.startIntakeRetraction();
        //right bumper to un-intake in case stuff gets stuck or something
//        robot.intakeModule.intakePow = toNum(gamepad1.left_bumper) - toNum(gamepad1.right_bumper);
//        robot.intakeModule.doorOpen = gamepad1.a;
//        robot.intakeModule.extenderPos = gamepad1.right_trigger;
    }

    private OuttakeModule.VerticalSlideLevel level = OuttakeModule.VerticalSlideLevel.DOWN;

    private void updateOuttakeStates() {
        if (gamepad2.dpad_left)
            robot.outtakeModule.setLevel(OuttakeModule.VerticalSlideLevel.DOWN);
        if (gamepad2.dpad_up)
            robot.outtakeModule.setLevel(OuttakeModule.VerticalSlideLevel.MID);
        if (gamepad2.dpad_right)
            robot.outtakeModule.setLevel(OuttakeModule.VerticalSlideLevel.TOP);

        if (gamepad2.left_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_INWARDS);
        else if (gamepad2.right_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS);

//        robot.outtakeModule.extendHopper = linkageToggle.isToggled(gamepad1.left_bumper);
//        robot.outtakeModule.dumpHopper = hopperPivotToggle.isToggled(gamepad1.right_bumper);
//
//        double rotateHopperChange = gamepad2.right_stick_x / 200000;
//        robot.outtakeModule.rotateHopper = Range.clip(robot.outtakeModule.rotateHopper + rotateHopperChange, 0, 1);
//
//        robot.outtakeModule.slideTargetPos = Range.clip(robot.outtakeModule.slideTargetPos + (toNum(gamepad2.dpad_down) - toNum(gamepad2.dpad_up)) / 1, -330, 0);
//
//        if (gamepad2.y && !prevY) {
//            robot.outtakeModule.currentlyDoingAction = true;
//        }
//        prevY = gamepad2.y;
    }
}
