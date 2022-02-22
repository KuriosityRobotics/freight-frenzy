package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class PlebIntakeLinkagesTuner extends LinearOpMode {
    Servo left;
    Servo right;

    double leftPos = 0.5;
    double rightPos = 0.6860951;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(Servo.class, "extenderLeft");
        right = hardwareMap.get(Servo.class, "extenderRight");

        waitForStart();

        while (opModeIsActive()) {
            leftPos += gamepad1.left_stick_y * 0.0005;
            rightPos += gamepad1.right_stick_y * 0.0005;
//            left.setPosition(leftPos);
            right.setPosition(rightPos);

            telemetry.addData("left pos: ", leftPos);
            telemetry.addData("right pos: ", rightPos);
            telemetry.update();
        }
    }
}
