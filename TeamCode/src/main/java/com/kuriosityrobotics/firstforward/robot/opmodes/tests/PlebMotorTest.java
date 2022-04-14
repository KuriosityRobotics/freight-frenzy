package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//@Disabled
public class PlebMotorTest extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    private AnalogInput distanceSensor;

    @Override
    public void runOpMode() {
//        intake = hardwareMap.dcMotor.get("intake");
//        intake = hardwareMap.dcMotor.get("carousel");
//        intake = hardwareMap.dcMotor.get("carousel");
        motor1 = hardwareMap.dcMotor.get("fLeft");
        motor2 = hardwareMap.dcMotor.get("fRight");
        motor3 = hardwareMap.dcMotor.get("bLeft");
        motor4 = hardwareMap.dcMotor.get("bRight");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //distanceSensor = hardwareMap.get(AnalogInput.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            win();
        }
    }

    public void win() {
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(-gamepad1.left_stick_y);
        motor3.setPower(gamepad1.left_stick_y);
        motor4.setPower(-gamepad1.left_stick_y);

        //telemetry.addData("motor pos: ", "" + motor.getCurrentPosition());
        //telemetry.addData("Distance sensor reading: ", "" +  distanceSensor.getVoltage());
        telemetry.update();
    }
}
