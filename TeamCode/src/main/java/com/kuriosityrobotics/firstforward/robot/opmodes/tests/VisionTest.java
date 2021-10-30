package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        var cam = new ManagedCamera("Webcam 1", hardwareMap, new VuforiaConsumer() {
            VuforiaTrackable stoneTarget;

            @Override
            public void setup(VuforiaLocalizer vuforia) {
                Log.d("VuforiaConsumer", "setup() called");
                VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("UltimateGoal");

                stoneTarget = targetsSkyStone.get(0);

            }

            @Override
            public void update() {
                Log.d("VisionTest->VuforiaConsumer", String.valueOf(((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()));
            }
        }, frame -> Log.d("VisionTest", String.valueOf(frame.width())));


        waitForStart();

        while (opModeIsActive()) ;

    }
}