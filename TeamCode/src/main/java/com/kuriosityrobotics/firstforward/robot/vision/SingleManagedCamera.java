package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.VUFORIA_LICENCE_KEY;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;

public final class SingleManagedCamera {
    private final VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;
    public static boolean vuforiaActive = true;
    private final List<OpenCvConsumer> openCvConsumers;
    private final WebcamName cameraName;
    private VuforiaLocalizer vuforia;

    public SingleManagedCamera(WebcamName cameraName, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);

        this.cameraName = cameraName;
        initializeVuforia(cameraName);
    }

    private void initializeVuforia(WebcamName webcamName) {
        if (vuforia != null) {
            vuforia.close();
            vuforia = null;
        }
        if (openCvCamera != null) {
            openCvCamera.closeCameraDevice();
            openCvCamera = null;
        }

        if (vuforiaConsumer != null) {
            // setup vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraName = webcamName;
            parameters.useExtendedTracking = false;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            vuforiaConsumer.setup(vuforia);
            openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);

        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName);
        }

        // set stuff up so opencv can also run
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

                openCvCamera.setPipeline(new CameraConsumerProcessor());
                try {
                    openCvCamera.startStreaming(640, 480);
                } catch(Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onError(int errorCode) {
                Log.d("Managed Camera", "Error: " + errorCode);
            }
        });
    }

    public void close() {
        this.vuforia.close();
        this.openCvCamera.closeCameraDevice();
    }

    public SingleManagedCamera(WebcamName webcamName, OpenCvConsumer... openCvConsumers) {
        this(webcamName, null, openCvConsumers);
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            if (vuforiaActive) {
                Coroutine<VuforiaConsumer, Void> vuforiaCoro = first(consume((VuforiaConsumer::update)));
                // !!
                Coroutine<OpenCvConsumer, Void> openCvCoro = first(consume((OpenCvConsumer consumer) -> { //!!
//                    Mat matCopy = input.clone();
                    consumer.processFrame(input);
//                    matCopy.release(); // c++ moment
                }));

                // distribute the data
                CoroutineScope.launch(scope ->
                {
                    if (vuforiaConsumer != null) {
                        vuforiaCoro.runAsync(scope, vuforiaConsumer);
                    }
                    openCvConsumers.forEach(consumer -> openCvCoro.runAsync(scope, consumer));
                });
            }

            return input;
        }
    }
}
