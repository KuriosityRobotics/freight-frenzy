package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class KalmanTest extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap, telemetry, this, true);
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }
        waitForStart();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
            updateIntakeStates();
            updateOuttakeStates();
            updateCarouselStates();
        }
    }

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
        robot.intakeModule.intakePower = (Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0);

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
        }
    }

    Button dpad_up = new Button();

    private void updateOuttakeStates() {
        if ((gamepad2.left_bumper || gamepad2.right_bumper) && robot.outtakeModule.targetState != OuttakeModule.OuttakeState.COLLAPSE)
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.COLLAPSE;

        if (dpad_up.isSelected(gamepad2.dpad_up)) {
            if (robot.outtakeModule.targetState != OuttakeModule.OuttakeState.RAISE && robot.outtakeModule.targetState != OuttakeModule.OuttakeState.EXTEND) {
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.RAISE;
            } else {
                if (robot.outtakeModule.targetSlideLevel == OuttakeModule.VerticalSlideLevel.TOP) {
                    robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP_TOP;
                } else {
                    robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
                }
            }
        } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.MID;

            if (robot.outtakeModule.targetState != OuttakeModule.OuttakeState.EXTEND) {
                robot.outtakeModule.targetState = OuttakeModule.OuttakeState.RAISE;
            }
        } else if (gamepad2.dpad_down) {
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.DOWN;
            robot.outtakeModule.skipToCollapse();
        }

        boolean y = gamepad2.y,
                x = gamepad2.x,
                b = gamepad2.b;

        if (x || b) {
            if (x) {
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.LEFT;
            }
            if (b) {
                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.RIGHT;
            }

            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.MID;
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
        }

        if (y) {
            robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
            robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.TOP;
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
        }
    }

    private void updateCarouselStates() {
        if (gamepad2.right_trigger > 0.01) {
            robot.carouselModule.clockwise = true;
        } else if (gamepad2.left_trigger > 0.01) {
            robot.carouselModule.clockwise = false;
        }
        robot.carouselModule.spin = gamepad2.left_trigger > 0.01 || gamepad2.right_trigger > 0.01;
    }
}
