package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.detector.YoloV5Classifier;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    @Override
    public void runOpMode() {
        var detector = new CargoDetectorConsumer(() -> new Pose(0, 12.5, 0));
        var managedCamera = new ManagedCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);

        waitForStart();
        while (opModeIsActive()) {
            detector.getTelemetryData().forEach(telemetry::addLine);
            telemetry.update();
            sleep(100);
        }
    }
}
