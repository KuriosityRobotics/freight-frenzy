package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
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
import java.util.Arrays;
import java.util.List;

import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;

public final class ManagedCamera implements Telemeter {
    private static final String VUFORIA_LICENCE_KEY =
            "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
                    "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
                    "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
                    "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
                    "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
                    "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
                    "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";

    private VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;

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

        activateCamera(this.cameraName2);
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

    @Override
    public String getName() {
        return "ManagedCamera";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            //Log.e("ManagedCamera", String.format("processFrame() called at %d", SystemClock.currentThreadTimeMillis()));

            Coroutine<VuforiaConsumer, Void> vuforiaCoro = first(consume((VuforiaConsumer::update)));
            //!!
            // c++ moment
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

            return input;
        }
    }
}
