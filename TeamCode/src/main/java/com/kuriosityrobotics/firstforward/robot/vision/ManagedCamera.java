package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.VUFORIA_LICENCE_KEY;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;

public final class ManagedCamera {
    private final VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;
    boolean vuforiaActive = true;
    private final List<OpenCvConsumer> openCvConsumers;
    private final WebcamName cameraName;
    private VuforiaLocalizer vuforia;

    public ManagedCamera(WebcamName cameraName, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
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

            // TODO: when this is interrupted we get really weird behavior. sometimes we catch an exception,
            // sometimes the entire rc crashes
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            vuforiaConsumer.setup(vuforia);
         /*   var exposureControl = vuforia.getCamera().getControl(WhiteBalanceControl.class);

            vuforia.getCamera().getControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
            exposureControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            exposureControl.setWhiteBalanceTemperature(4500);*/
            vuforia.getCamera().getControl(FocusControl.class).setMode(FocusControl.Mode.Infinity);
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
                    openCvCamera.startStreaming(800, 448);
                } catch(Exception e) {
                    Log.e("VisionThread", "An error occured! " + e);
                }
            }

            @Override
            public void onError(int errorCode) {
                Log.d("Managed Camera", "Error: " + errorCode);
            }
        });
    }

    public ManagedCamera(WebcamName webcamName, OpenCvConsumer... openCvConsumers) {
        this(webcamName, null, openCvConsumers);
    }

    public void close() {
//        this.openCvCamera.closeCameraDevice();
        this.vuforia.close();
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            if (input.empty())
                return input;

            if (vuforiaActive) {
                Coroutine<VuforiaConsumer, Void> vuforiaCoro = first(consume((VuforiaConsumer::update)));
                // !!
                Coroutine<OpenCvConsumer, Void> openCvCoro = first(consume((OpenCvConsumer consumer) -> { //!!
                    Mat matCopy = input.clone();
                    consumer.processFrame(matCopy);
                    matCopy.release(); // c++ moment
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
