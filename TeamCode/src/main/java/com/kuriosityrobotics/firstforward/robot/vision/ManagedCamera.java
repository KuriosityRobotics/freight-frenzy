package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.VUFORIA_LICENCE_KEY;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;
import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;
import android.util.Log;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

public final class ManagedCamera {
    private final VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;

    public static boolean vuforiaActive = true;
    private static boolean vuforiaInitialisedYet = false;

    private final List<OpenCvConsumer> openCvConsumers;

    private final WebcamName cameraName1;
    private final WebcamName cameraName2;

    private SwitchableCamera switchableCamera;
    private WebcamName activeCameraName;
    private VuforiaLocalizer vuforia;

    public ManagedCamera(WebcamName cameraName1, WebcamName cameraName2, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);

        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
        SwitchableCameraName switchableCameraName = ClassFactory.getInstance()
                .getCameraManager()
                .nameForSwitchableCamera(this.cameraName1, this.cameraName2);
        initializeVuforia(switchableCameraName);
        activateCamera(this.cameraName2);
    }

    private void initializeVuforia(SwitchableCameraName switchableCameraName) {
        if (vuforiaInitialisedYet) {
            throw new RuntimeException("You cannot initialize multiple vuforias at once!");
        }

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
            parameters.cameraName = switchableCameraName;
            parameters.useExtendedTracking = false;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            switchableCamera = (SwitchableCamera) vuforia.getCamera();

            vuforiaConsumer.setup(vuforia);
            openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);

            try {
                // hack moment(we're passing in a SwitchableCamera(not a Camera), which causes OpenCV to mald even though it shouldn't because of polymorphism)
                // anyways enough of this rant
                Class<?> aClass = Class.forName("org.openftc.easyopencv.OpenCvVuforiaPassthroughImpl");
                Field isWebcamField = aClass.getDeclaredField("isWebcam");
                isWebcamField.setAccessible(true);
                isWebcamField.set(openCvCamera, true);
            } catch (NoSuchFieldException | IllegalAccessException | ClassNotFoundException e) {
                Log.e("Switchable Cameras: ", "cannot set isWebcam ", e);
            }

        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName2);
        }

        // set stuff up so opencv can also run
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                activateCamera(cameraName2);
                openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

                openCvCamera.setPipeline(new CameraConsumerProcessor());
                openCvCamera.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                Log.d("Managed Camera", "Error: " + errorCode);
            }
        });

        vuforiaInitialisedYet = true;
    }

    public ManagedCamera(WebcamName webcamName1, WebcamName webcamName2, OpenCvConsumer... openCvConsumers) {
        this(webcamName1, webcamName2, null, openCvConsumers);
    }

    public void activateCamera(WebcamName cameraName) {
        if (this.activeCameraName == cameraName) {
            return;
        }

        if (switchableCamera == null) {
            Log.e("ManagedCamera", "Not a switchable camera");
            return;
        }

        activateCamera(cameraName);
        this.activeCameraName = cameraName;
    }

    public CameraName getActiveCameraName() {
        return activeCameraName;
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
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

    public void onClose() {
        vuforiaInitialisedYet = false;
        for (OpenCvConsumer o : openCvConsumers) {
            o.deactivate();
        }
        openCvCamera.stopStreaming();
    }
}
