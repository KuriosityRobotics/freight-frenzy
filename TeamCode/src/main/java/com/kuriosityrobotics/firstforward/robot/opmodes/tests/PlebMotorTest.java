package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

@TeleOp
@Disabled
public class PlebMotorTest extends LinearOpMode {
    private DcMotor motor;
    private AnalogInput distanceSensor;

    @Override
    public void runOpMode() {
//        intake = hardwareMap.dcMotor.get("intake");
//        intake = hardwareMap.dcMotor.get("carousel");
//        intake = hardwareMap.dcMotor.get("carousel");
        motor = hardwareMap.dcMotor.get("fLeft");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        distanceSensor = hardwareMap.get(AnalogInput.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            win();
        }
    }

    public void win() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("motor pos: ", "" + motor.getCurrentPosition());
        telemetry.addData("Distance sensor reading: ", "" +  distanceSensor.getVoltage());
        telemetry.update();
    }
}
