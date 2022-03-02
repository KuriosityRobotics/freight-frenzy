package com.kuriosityrobotics.firstforward.robot.opmodes.tests.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class OuttakeTurretTuner extends LinearOpMode {
    Servo servo;

    double servoPos = 0.5;

    @Override
    public void runOpMode() {
//        servo = hardwareMap.servo.get("extenderRight");
        servo = hardwareMap.servo.get("webcamPivot");

        waitForStart();

        servo.setPosition(servoPos);

        while (opModeIsActive()) {
            servoPos += gamepad1.right_stick_y * 0.0003;
            servo.setPosition(servoPos);

            telemetry.addData("extender pos: ", servoPos);
            telemetry.update();
        }
    }
}
