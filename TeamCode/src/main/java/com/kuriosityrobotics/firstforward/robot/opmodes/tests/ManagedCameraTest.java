package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
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
        long updateTime = 0;
        long lastLoopTime = 0;

        OpenCVDumper dumper = new OpenCVDumper(true);
        TelemetryDump dump = new TelemetryDump(telemetry, false);
        LocalizationConsumer vufConsumer = new LocalizationConsumer();
        ManagedCamera cam = new ManagedCamera("Webcam 2", hardwareMap, vufConsumer, dumper);

        waitForStart();

        while (opModeIsActive()) {
            // yeet
            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            telemetry.addData("Status: ", "Still running");
            telemetry.addData("Update Time: ", "hella " + updateTime);
            telemetry.update();
        }
    }
}
