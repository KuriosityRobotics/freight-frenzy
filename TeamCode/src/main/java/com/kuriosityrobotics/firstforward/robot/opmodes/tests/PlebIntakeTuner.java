package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class PlebIntakeTuner extends LinearOpMode {
    Servo extender;

    double extenderPos = 0.5;

    @Override
    public void runOpMode() {
        extender = hardwareMap.servo.get("extenderLeft");

        waitForStart();

        extender.setPosition(extenderPos);

        while (opModeIsActive()) {
            extenderPos += gamepad1.right_stick_y * 0.00005;
            extender.setPosition(extenderPos);

            telemetry.addData("extender pos: ", extenderPos);
            telemetry.update();
        }
    }
}
