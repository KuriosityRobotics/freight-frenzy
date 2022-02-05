package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp
public class PlebMotorTest extends LinearOpMode {
    private DcMotor intake;

    @Override
    public void runOpMode() {
//        intake = hardwareMap.dcMotor.get("intake");
        intake = hardwareMap.dcMotor.get("carousel");
//        intake = hardwareMap.dcMotor.get("carousel");

        waitForStart();

        while (opModeIsActive()) {
            win();
        }
    }

    public void win() {
        intake.setPower(gamepad1.left_stick_y);
        telemetry.addData("intake pos: ", "" + intake.getCurrentPosition());
        telemetry.update();
    }
}
