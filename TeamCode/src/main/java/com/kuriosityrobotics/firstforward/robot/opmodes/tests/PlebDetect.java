package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import com.kuriosityrobotics.firstforward.robot.vision.SingleManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    public TeamMarkerDetector detector;
    public SingleManagedCamera singleManagedCamera;

    @Override
    public void runOpMode() {
        this.detector =  new TeamMarkerDetector();
        this.singleManagedCamera = new SingleManagedCamera(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);

        waitForStart();
        while (opModeIsActive()) {
            // yeet
            telemetry.addData("Location: ", detector.getLocation());
            telemetry.update();
            sleep(100);
        }
    }
}
