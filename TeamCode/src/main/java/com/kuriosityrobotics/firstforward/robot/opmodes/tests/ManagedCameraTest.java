package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.TelemetryDump;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCVDumper;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ManagedCameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVDumper dumper = new OpenCVDumper(true);
        TelemetryDump dump = new TelemetryDump(telemetry, false);
        LocalizationConsumer vufConsumer = new LocalizationConsumer();
        ManagedCamera cam = new ManagedCamera("Webcam 2", hardwareMap, vufConsumer, dumper);

        waitForStart();

        while (opModeIsActive()) {
            // yeet
            dump.update();
            telemetry.addData("Status: ", "Still running");
            telemetry.update();
        }
    }
}
