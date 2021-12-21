package com.kuriosityrobotics.firstforward.robot.vision;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.VUFORIA_LICENCE_KEY;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;
import de.esoco.coroutine.*;

import android.util.Log;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class ManagedCamera {
    private VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;

    private boolean vuforiaInitialisedYet;
    public static boolean vuforiaActive = true;

    private List<OpenCvConsumer> openCvConsumers;

    private WebcamName cameraName1;
    private WebcamName cameraName2;

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
        initializeVulforia(switchableCameraName);
        switchableCamera.setActiveCamera(this.cameraName1);
        activateCamera(this.cameraName1);
    }

    private void initializeVulforia(SwitchableCameraName switchableCameraName) {
        if (vuforia != null) {
            vuforia.close();
            vuforia = null;
        }
        if (openCvCamera != null) {
            openCvCamera.closeCameraDevice();
            openCvCamera = null;
        }

        if (vuforiaConsumer != null) {
<<<<<<< HEAD
            // setup vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraName = switchableCameraName;
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
=======
            if(!vuforiaInitialisedYet) {
                // setup vuforia
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                parameters.cameraName = cameraName;

                VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
                vuforiaConsumer.setup(vuforia);
                openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);

                vuforiaInitialisedYet = true;
            } else {
                // control hub does not like multiple vuforias, so don't try spawning more than 1 Managed Camera
                throw new RuntimeException("ManagedCamera(String, HardwareMap, VuforiaConsumer, ...) constructor called multiple times.  Running more than one instance of Vuforia isn't supported and will lead to a crash.");
>>>>>>> 9b4a14a (small changes)
            }

        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName2);
        }

        // set stuff up so opencv can also run
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

                openCvCamera.setPipeline(new CameraConsumerProcessor());
                openCvCamera.startStreaming(1920, 1080);
            }

            @Override
            public void onError(int errorCode) {
                Log.d("Managed Camera", "Error: " + errorCode);
            }
        });
    }

    public void activateCamera(WebcamName cameraName) {
        if (this.activeCameraName == cameraName) {
            return;
        }

        if (switchableCamera == null) {
            Log.e("ManagedCamera", "Not a switchable camera");
            return;
        }

        this.switchableCamera.setActiveCamera(cameraName);
        this.activeCameraName = cameraName;
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
}
