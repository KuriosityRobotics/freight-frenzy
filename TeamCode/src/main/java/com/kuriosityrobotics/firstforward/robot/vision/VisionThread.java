package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.graphics.Bitmap;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.CameraConsumer;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.LinkedBlockingQueue;

import de.esoco.coroutine.CoroutineScope;

public class VisionThread implements Runnable {
    private final String configLocation;
    private final Robot robot;
    private final LinkedBlockingQueue<CameraConsumer> consumers = new LinkedBlockingQueue<>(); // lmao

    public VisionThread(String configLocation, Robot robot) {
        this.configLocation = configLocation;
        this.robot = robot;
    }

    public void registerConsumer(CameraConsumer consumer) {
        consumers.add(consumer);
    }


    @Override
    public void run() {
        VuforiaLocalizer lem = ClassFactory.getInstance().createVuforia(new VuforiaLocalizer.Parameters());

        while (robot.running()) {
            lem.getFrameOnce(Continuation.create(ThreadPool.getDefault(), mon -> {
                lem.convertFrameToBitmap(mon);
                Mat mat = new Mat();
                Bitmap bmp32 = lem.convertFrameToBitmap(mon).copy(Bitmap.Config.ARGB_8888, true);
                Utils.bitmapToMat(bmp32, mat);


                var coro = first(consume((CameraConsumer consumer) -> { //!!
                    var matCopy = mat.clone();
                    consumer.processFrame(matCopy);
                    matCopy.release(); // c++ moment
                }));
                CoroutineScope.launch(scope ->
                        consumers.forEach(consumer -> coro.runAsync(scope, consumer)));

                mat.release();

            }));
        }
    }
}
