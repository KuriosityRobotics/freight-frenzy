package com.kuriosityrobotics.firstforward.robot.opmodes.tests.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class PlebOuttakeHopperDumpTuner extends LinearOpMode {
    Servo dump;

    double dumpPos = 0.85;

    @Override
    public void runOpMode() {
        dump = hardwareMap.get(Servo.class, "outtakeClamp");

        waitForStart();

        dump.setPosition(dumpPos);

        while (opModeIsActive()) {
            dumpPos += gamepad1.right_stick_y * 0.00005;
            dump.setPosition(dumpPos);

            telemetry.addData("dump motor pos: ", dumpPos);
            telemetry.update();
        }
    }
}
