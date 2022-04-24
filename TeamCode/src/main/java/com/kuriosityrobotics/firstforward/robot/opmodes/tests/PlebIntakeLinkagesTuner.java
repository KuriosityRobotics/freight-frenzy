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

    public static final double INTAKE_RIGHT_EXTENDED_POS = 0.0168;
    public static final double INTAKE_RIGHT_IDLE_POS = 0.5121062;
    public static final double INTAKE_RIGHT_RETRACTED_POS = 0.6860951;
    public static final double INTAKE_LEFT_EXTENDED_POS = 0.949761;
    public static final double INTAKE_LEFT_RETRACTED_POS = 0.289751;
    public static final double INTAKE_LEFT_IDLE_POS = INTAKE_LEFT_RETRACTED_POS + (INTAKE_RIGHT_RETRACTED_POS - INTAKE_RIGHT_IDLE_POS);

    double leftPos = INTAKE_LEFT_EXTENDED_POS;
    double rightPos = INTAKE_RIGHT_EXTENDED_POS;

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
