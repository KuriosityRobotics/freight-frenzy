package com.kuriosityrobotics.firstforward.robot.opmodes;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

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
        robot.intakeModule.intakePower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.requestRetraction();
        }
    }

    private void updateOuttakeStates() {

        if (gamepad2.left_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_INWARDS);
        else if (gamepad2.right_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS);

        if (gamepad2.dpad_up) {
            robot.outtakeModule.raise();
            robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.TOP);
        } else if (gamepad2.dpad_right) {
            robot.outtakeModule.raise();
            robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.MID);
        } else if (gamepad2.dpad_down) {
            robot.outtakeModule.raise();
            robot.outtakeModule.setSlideLevel(OuttakeModule.VerticalSlideLevel.DOWN);
        }
        if(gamepad2.b)
            robot.outtakeModule.pivotIn();
        else if (gamepad2.y)
            robot.outtakeModule.pivotStraight();
        else if (gamepad2.x)
            robot.outtakeModule.pivotRight();
//        else if (gamepad2.b)
//            robot.outtakeModule.pivot270();

    }

    private void updateCarouselStates() {
        if (gamepad2.right_trigger > 0.01){
            robot.carouselModule.clockwise = true;
        } else if (gamepad2.left_trigger>0.01) {
            robot.carouselModule.clockwise = false;
        }
        robot.carouselModule.spin = gamepad2.left_trigger > 0.01 || gamepad2.right_trigger > 0.01;
    }
}
