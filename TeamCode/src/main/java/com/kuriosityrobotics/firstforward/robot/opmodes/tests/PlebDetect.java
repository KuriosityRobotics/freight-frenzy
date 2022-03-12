package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    @Override
    public void runOpMode() {
        var detector = new CargoDetectorConsumer(LocationProvider.of(Pose.ZERO, Pose.ZERO));
        var managedCamera = new ManagedCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), LocationProvider.of(Pose.ZERO, Pose.ZERO), detector);
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
