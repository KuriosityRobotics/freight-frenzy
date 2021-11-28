package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.modules.IntakeModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class PlebRPMDeterminer extends LinearOpMode {
    private IntakeModule intake;

    @Override
    public void runOpMode() {
        intake = new IntakeModule(hardwareMap, true);

        waitForStart();

        while (opModeIsActive()) {
            win();
        }
    }

    public void win() {
        intake.intakePower = gamepad1.left_stick_y;
        intake.update();
        intake.getTelemetryData().forEach(telemetry::addLine);
        telemetry.update();
    }
}
