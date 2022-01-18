package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    public TeamMarkerDetector detector;
    public ManagedCamera managedCamera;

    @Override
    public void runOpMode() {
        this.detector =  new TeamMarkerDetector();
        this.managedCamera = new ManagedCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.get(WebcamName.class, "Webcam 2"), detector);

        waitForStart();
        while (opModeIsActive()) {
            // yeet
            telemetry.addData("Location: ", detector.getLocation());
            telemetry.update();
            sleep(100);
        }
    }
}
