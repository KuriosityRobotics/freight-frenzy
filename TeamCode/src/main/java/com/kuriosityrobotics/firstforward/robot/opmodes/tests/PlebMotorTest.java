package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp
public class PlebMotorTest extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
//        intake = hardwareMap.dcMotor.get("intake");
//        intake = hardwareMap.dcMotor.get("carousel");
//        intake = hardwareMap.dcMotor.get("carousel");
        motor = hardwareMap.dcMotor.get("fLeft");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            win();
        }
    }

    public void win() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("motor pos: ", "" + motor.getCurrentPosition());
        telemetry.update();
    }
}
