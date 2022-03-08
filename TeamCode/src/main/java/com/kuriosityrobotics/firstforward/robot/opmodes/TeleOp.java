package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap, telemetry, this, true);
            robot.resetPose(new Pose(28, 60, Math.toRadians(-90)));
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
            updateCapStates();
            updateCarouselStates();
        }
    }

    private void updateDrivetrainStates() {
        double yMov = Math.signum(gamepad1.left_stick_y) * -Math.pow(gamepad1.left_stick_y, 2);
        double xMov = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double turnMov = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            yMov /= 2;
            xMov /= 2;
            turnMov /= 2;
        }

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
        }
    }

    Button dpad_up = new Button();
    Button yButton = new Button();
    Button lBump = new Button();
    Button xGamepad2 = new Button();
    Button yGamepad2 = new Button();
    Button bGamepad2 = new Button();
    Button lTriggerGamepad2 = new Button();

    private void updateOuttakeStates() {
        if ((gamepad1.right_bumper || gamepad2.right_bumper) && robot.outtakeModule.targetState != OuttakeModule.OuttakeState.COLLAPSE)
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
            robot.outtakeModule.skipToCollapse();
        }

        boolean y = gamepad2.y,
                x = gamepad2.x,
                b = gamepad2.b;

        if(xGamepad2.isSelected(gamepad2.x)){
            robot.outtakeModule.isShared = true;
            robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_LEFT;
        }else if(bGamepad2.isSelected(gamepad2.b)){
            robot.outtakeModule.isShared = true;
            robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_RIGHT;
        }else if(yGamepad2.isSelected(gamepad2.y)){
            robot.outtakeModule.isShared = false;
            robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
        }else if(lTriggerGamepad2.isSelected(gamepad2.left_trigger > 0)){
            robot.outtakeModule.isShared = true;
            robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
        }
        if (x || b) {
//            if (x) {
//                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_LEFT;
//            }
//            if (b) {
//                robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.SHARED_RIGHT;
//            }

            if (robot.outtakeModule.targetState != OuttakeModule.OuttakeState.EXTEND)
                robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.DOWN;

            robot.outtakeModule.capping = false;
            robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
        }

        if (yButton.isSelected(y)) {
            if (robot.outtakeModule.targetState == OuttakeModule.OuttakeState.EXTEND) {
                if (robot.outtakeModule.targetTurret == OuttakeModule.TurretPosition.STRAIGHT) {
                    robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.ALLIANCE_LOCK;
                } else {
                    robot.outtakeModule.targetTurret = OuttakeModule.TurretPosition.STRAIGHT;
                }
            } else {
                robot.outtakeModule.defaultFullExtend();
            }
        }
    }

    private void updateCapStates() {
        if (lBump.isSelected(gamepad2.left_bumper)) {
            switch (robot.outtakeModule.targetState) {
                case COLLAPSE:
                    robot.outtakeModule.capping = false;
                    robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.DOWN;
                    robot.outtakeModule.targetState = OuttakeModule.OuttakeState.EXTEND;
                    break;
                case EXTEND:
                    if (!robot.outtakeModule.capping) {
                        robot.outtakeModule.capping = true;
                        robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP;
                    } else {
                        robot.outtakeModule.targetSlideLevel = OuttakeModule.VerticalSlideLevel.CAP_DROP;
                    }
                    break;
            }
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
