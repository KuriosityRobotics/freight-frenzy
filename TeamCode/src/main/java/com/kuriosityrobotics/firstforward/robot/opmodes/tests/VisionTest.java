package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.util.Log;

import androidx.annotation.NonNull;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.*;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import de.esoco.coroutine.CoroutineScope;
import javassist.NotFoundException;

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