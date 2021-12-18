package com.kuriosityrobotics.firstforward.robot.opmodes;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();
    Button switchCameraButton = new Button();

    private boolean prevY = false;

    @Override
    public void runOpMode() {
        try {
            // TODO: change
            robot = new Robot(hardwareMap, telemetry, this, Autonomous.PARK);
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
            updateCarouselStates();
            //robot.drivetrain.setBrakePose(new Pose(10,0,0));
        }
    }

    private final double EPSILON = 0.1;

    private void updateDrivetrainStates() {
        double yMov = Math.signum(gamepad1.left_stick_y) * -Math.pow(gamepad1.left_stick_y, 2);
        double xMov = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double turnMov = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            yMov /= 2;
            xMov /= 2;
            turnMov /= 2;
        }

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = Math.abs(gamepad2.left_stick_y) > EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.retractIntake = true;
        }
    }

    private OuttakeModule.VerticalSlideLevel level = OuttakeModule.VerticalSlideLevel.DOWN;

    private void updateOuttakeStates() {
        if (gamepad2.dpad_down)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.DOWN;
        if (gamepad2.dpad_up)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.TOP;
        if (gamepad2.dpad_right)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.MID;

        if (gamepad2.left_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_INWARDS);
        else if (gamepad2.right_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS);
    }

    private void updateCarouselStates() {
        robot.carouselModule.spin = gamepad2.x;
    }
}
