package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeTurretTuner extends LinearOpMode {
    Servo pivot;

    double pivotPos = 0.906181;

    @Override
    public void runOpMode() {
        pivot = hardwareMap.get(Servo.class, "outtakeTurret");

        waitForStart();

        pivot.setPosition(pivotPos);

        while (opModeIsActive()) {
            pivotPos += gamepad1.right_stick_y * 0.00005;
            pivot.setPosition(pivotPos);

            telemetry.addData("turret pos: ", pivotPos);
            telemetry.update();
        }
    }
}
