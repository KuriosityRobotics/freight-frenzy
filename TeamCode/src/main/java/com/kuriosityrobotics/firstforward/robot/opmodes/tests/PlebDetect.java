package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.sensors.FileDump;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.TeamMarkerDetection;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import de.esoco.coroutine.Coroutine;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PlebDetect extends LinearOpMode {
    public TeamMarkerDetection detector;
    public ManagedCamera managedCamera;

    @Override
    public void runOpMode() {
        this.detector =  new TeamMarkerDetection();
        this.managedCamera = new ManagedCamera("Webcam 1", hardwareMap, detector);

        waitForStart();
        while (opModeIsActive()) {
            // yeet
            telemetry.addData("Location: ", detector.getLocation());
            telemetry.update();
            sleep(100);
        }
    }
}
