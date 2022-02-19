package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class PlebServoTuner extends LinearOpMode {
    Servo servo;

    double servoPos = 0.5;

    @Override
    public void runOpMode() {
        Orientation.getRotationMatrix()
//        servo = hardwareMap.servo.get("extenderRight");
        // turret -/ pivot
        var a = hardwareMap.servo.get("0");
        var b = hardwareMap.servo.get("1");
        var c = hardwareMap.servo.get("2");
        var d = hardwareMap.servo.get("3");
        var e = hardwareMap.servo.get("4");
        var f = hardwareMap.servo.get("5");
        var aa = hardwareMap.servo.get("0a");
        var bb = hardwareMap.servo.get("1a");
        var cc = hardwareMap.servo.get("2a");
        var dd = hardwareMap.servo.get("3a");
        var ee = hardwareMap.servo.get("4a");
        var ff = hardwareMap.servo.get("5a");

        Servo current = a;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                current = a;
            else if (gamepad1.b)
                current = b;
            else if (gamepad1.x)
                current = c;
            else if(gamepad1.y)
                current = d;
            else if(gamepad1.right_bumper)
                current = e;
            else if(gamepad1.start)
                current = f;

           else if (gamepad1.dpad_down)
                current = aa;
            else if (gamepad1.dpad_right)
                current = bb;
            else if (gamepad1.dpad_left)
                current = cc;
            else if(gamepad1.dpad_up)
                current = dd;
            else if(gamepad1.left_bumper)
                current = ee;
            else if(gamepad1.back)
                current = ff;

            servoPos += gamepad1.right_stick_y * 0.0003;
            current.setPosition(servoPos);

            telemetry.addData("servo pos: ", servoPos);
            telemetry.update();
        }
    }
}
