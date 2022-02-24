package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    @Override
    public void runOpMode() {
        var detector = new CargoDetectorConsumer(() -> new Pose(0, 12.5, 0));
        var managedCamera = new ManagedCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);
        var thread = new Thread(detector);
        thread.start();

        waitForStart();
        while (opModeIsActive()) {
            detector.getTelemetryData().forEach(telemetry::addLine);
            telemetry.update();
            sleep(100);
        }
        thread.interrupt();
    }
}
