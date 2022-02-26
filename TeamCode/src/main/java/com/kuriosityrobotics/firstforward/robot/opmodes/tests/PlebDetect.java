package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.CargoDetectorConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    @Override
    public void runOpMode() {
        var detector = new CargoDetectorConsumer(LocationProvider.of(Pose.ZERO, Pose.ZERO), PhysicalCamera.of(new Vector3D(0, 12.5, 0), new Rotation(new Vector3D(-1, 0, 0), PI/6)));
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
