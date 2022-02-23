package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class PlebCamRotationTuner extends LinearOpMode {
    private Servo camRotator;

    @Override
    public void runOpMode() throws InterruptedException {
        camRotator = hardwareMap.get(Servo.class, "webcamPivot");
        double pos = 0.399897;
        camRotator.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {
            pos += 0.00001 * gamepad1.left_stick_x;
            camRotator.setPosition(pos);
            telemetry.addData("Position", "hella " + pos);
        }
    }
}
