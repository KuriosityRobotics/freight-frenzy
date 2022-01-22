package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PlebEncoderDeterminer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx fLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "fLeft");
        DcMotorEx fRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "fRight");
        DcMotorEx bLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "bLeft");
        DcMotorEx bRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "bRight");

        Servo outtakeLinkage = hardwareMap.get(Servo.class, "outtakeLinkage");
        Servo pivot = hardwareMap.get(Servo.class, "pivot");
        Servo hopper = hardwareMap.get(Servo.class, "hopper");

        waitForStart();

        double linkPos = 0.2;
        double pivotPos = 0.2;
        double hopperpos = 0.2;

        while (opModeIsActive()) {
//            telemetry.addData("fleft", "hella" + fLeft.getVelocity());
//            telemetry.addData("fRight", "hella" + fRight.getVelocity());
//            telemetry.addData("bLeft", "hella" + bLeft.getVelocity());
//            telemetry.addData("bRight", "hella" + bRight.getVelocity());
            linkPos += gamepad1.left_stick_x * 0.0001;
            pivotPos += gamepad1.left_stick_y * 0.0001;
            hopperpos += gamepad1.right_stick_x * 0.0001;
            outtakeLinkage.setPosition(linkPos);
            pivot.setPosition(pivotPos);
            hopper.setPosition(hopperpos);
            telemetry.addData("linkage", "hella" + linkPos);
            telemetry.addData("pivot", "hella" + pivotPos);
            telemetry.addData("hopper", "hella" + hopperpos);
            telemetry.update();
        }
    }
}
