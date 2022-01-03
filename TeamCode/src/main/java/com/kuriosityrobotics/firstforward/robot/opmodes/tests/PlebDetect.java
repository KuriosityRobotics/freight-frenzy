package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    public TeamMarkerDetection detector;
    public ManagedCamera managedCamera;

    @Override
    public void runOpMode() {
        this.detector =  new TeamMarkerDetection();
        this.managedCamera = new ManagedCamera("Webcam 2", hardwareMap, detector);

        waitForStart();
        while (opModeIsActive()) {
            // yeet
            telemetry.addData("Location: ", detector.getLocation());
            telemetry.update();
            sleep(100);
        }
    }
}
