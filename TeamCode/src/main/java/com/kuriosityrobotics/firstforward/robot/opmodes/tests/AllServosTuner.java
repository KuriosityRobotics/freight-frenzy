package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class AllServosTuner extends LinearOpMode {
    Servo servo0;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    Servo servo5;

    Servo servo6;
    Servo servo7;
    Servo servo8;
    Servo servo9;
    Servo servo10;
    Servo servo11;

    double servoPos0 = 0.5;
    double servoPos1 = 0.5;
    double servoPos2 = 0.5;
    double servoPos3 = 0.5;
    double servoPos4 = 0.5;
    double servoPos5 = 0.5;

    double servoPos6 = 0.5;
    double servoPos7 = 0.5;
    double servoPos8 = 0.5;
    double servoPos9 = 0.5;
    double servoPos10 = 0.5;
    double servoPos11 = 0.5;

    @Override
    public void runOpMode() {
        servo0 = hardwareMap.servo.get("0"); //a
        servo1 = hardwareMap.servo.get("1"); //x
        servo2 = hardwareMap.servo.get("2"); //y
        servo3 = hardwareMap.servo.get("3"); //b
        servo4 = hardwareMap.servo.get("4"); //left
        servo5 = hardwareMap.servo.get("5"); //right

        servo6 = hardwareMap.servo.get("6"); //a
        servo7 = hardwareMap.servo.get("7"); //x
        servo8 = hardwareMap.servo.get("8"); //y
        servo9 = hardwareMap.servo.get("9"); //b
        servo10 = hardwareMap.servo.get("10"); //left
        servo11 = hardwareMap.servo.get("11"); //right

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                servoPos0 += .0003;
            }
            if (gamepad1.x){
                servoPos1 += .0003;
            }
            if (gamepad1.y){
                servoPos2 += .0003;
            }
            if (gamepad1.b){
                servoPos3 += .0003;
            }
            servoPos4 += gamepad1.left_stick_y * .0003;
            servoPos5 += gamepad1.right_stick_y * .0003;

            servo0.setPosition(servoPos0);
            servo1.setPosition(servoPos1);
            servo2.setPosition(servoPos2);
            servo3.setPosition(servoPos3);
            servo4.setPosition(servoPos4);
            servo5.setPosition(servoPos5);

            if (gamepad2.a){
                servoPos6 += .0003;
            }
            if (gamepad2.x){
                servoPos7 += .0003;
            }
            if (gamepad2.y){
                servoPos8 += .0003;
            }
            if (gamepad2.b){
                servoPos9 += .0003;
            }
            servoPos10 += gamepad2.left_stick_y * .0003;
            servoPos11 += gamepad2.right_stick_y * .0003;

            servo6.setPosition(servoPos6);
            servo7.setPosition(servoPos7);
            servo8.setPosition(servoPos8);
            servo9.setPosition(servoPos9);
            servo10.setPosition(servoPos10);
            servo11.setPosition(servoPos11);

            telemetry.addData("servoPos0: ", servoPos0);
            telemetry.addData("servoPos1: ", servoPos1);
            telemetry.addData("servoPos2: ", servoPos2);
            telemetry.addData("servoPos3: ", servoPos3);
            telemetry.addData("servoPos4: ", servoPos4);
            telemetry.addData("servoPos5: ", servoPos5);
            telemetry.addData("servoPos6: ", servoPos6);
            telemetry.addData("servoPos7: ", servoPos7);
            telemetry.addData("servoPos8: ", servoPos8);
            telemetry.addData("servoPos9: ", servoPos9);
            telemetry.addData("servoPos10: ", servoPos10);
            telemetry.addData("servoPos11: ", servoPos11);

            telemetry.update();
        }
    }
}
