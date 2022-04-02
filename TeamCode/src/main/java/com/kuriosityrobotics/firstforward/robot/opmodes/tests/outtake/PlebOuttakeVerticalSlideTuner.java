package com.kuriosityrobotics.firstforward.robot.opmodes.tests.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp
public class PlebOuttakeVerticalSlideTuner extends LinearOpMode {
    DcMotorEx slide;
    DcMotorEx slide2;

    int verticalSlideMotorPos = 0;

    @Override
    public void runOpMode() {
        slide = (DcMotorEx) hardwareMap.get(DcMotor.class, "otherLift");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lift");
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12, 0, 0, 20));
        slide2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12, 0, 0, 20));

        waitForStart();

        while (opModeIsActive()) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setTargetPosition(-100);
            slide2.setTargetPosition(100);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slide.setPower(1);
            slide2.setPower(1);

            telemetry.addData("verticalSlideMotorPos: ", -100);
            telemetry.addData("slide pos", slide.getCurrentPosition());
            telemetry.addData("slide pos", slide2.getCurrentPosition());
            telemetry.update();
        }
    }
}
