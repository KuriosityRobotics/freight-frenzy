package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PlebOuttakeHopperDumpTuner extends LinearOpMode {
    Servo dumpMotor;

    double dumpMotorPos = 0.5;

    @Override
    public void runOpMode() {
        dumpMotor = hardwareMap.get(Servo.class, "hopper");

        waitForStart();

        dumpMotor.setPosition(dumpMotorPos);

        while (opModeIsActive()) {
            dumpMotorPos += gamepad1.right_stick_y * 0.00005;
            dumpMotor.setPosition(dumpMotorPos);

            telemetry.addData("dump motor pos: ", dumpMotorPos);
            telemetry.update();
        }
    }
}
