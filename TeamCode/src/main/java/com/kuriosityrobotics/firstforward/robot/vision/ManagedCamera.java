package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Field;
import java.util.ArrayList;
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

    private SwitchableCamera switchableCamera;

    private boolean vuforiaInitialisedYet;

    private List<OpenCvConsumer> openCvConsumers;

    private WebcamName cameraNameFront;
    private WebcamName cameraNameLeft;

    private boolean isFrontCameraActive;

    public ManagedCamera(String camera1, String camera2, HardwareMap hardwareMap, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);
        this.cameraNameFront = hardwareMap.get(WebcamName.class, camera1);
        this.cameraNameLeft = hardwareMap.get(WebcamName.class, camera2);

        if (vuforiaConsumer != null) {
            if(!vuforiaInitialisedYet) {
                // setup vuforia
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
                parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(cameraNameFront, cameraNameLeft);

                VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
                vuforiaConsumer.setup(vuforia);
                switchableCamera = (SwitchableCamera) vuforia.getCamera();
                switchableCamera.setActiveCamera(cameraNameFront);
                this.isFrontCameraActive = true;
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

                vuforiaInitialisedYet = true;
            } else {
                // control hub does not like multiple vuforias, so don't try spawning more than 1 Managed Cameras
                throw new RuntimeException("ManagedCamera(String, HardwareMap, VuforiaConsumer, ...) constructor called multiple times.  Running more than one instance of Vuforia isn't supported and will lead to a crash.");
            }
        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraNameFront, cameraNameLeft);
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

    public ManagedCamera(String camera1, String camera2, HardwareMap hardwareMap, OpenCvConsumer... openCvConsumers) {
        this(camera1, camera2, hardwareMap, null, openCvConsumers);
    }

    @Override
    public List<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        if (switchableCamera.getActiveCamera() == cameraNameFront) {
            data.add("Active Camera: Front Webcam");
        } else if (switchableCamera.getActiveCamera() == cameraNameLeft) {
            data.add("Active Camera: Back Webcam");
        } else {
            data.add("Active Camera: no camera is active?");
        }

        return data;
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

    public void switchCameras() {
        if (isFrontCameraActive) {
            switchableCamera.setActiveCamera(cameraNameLeft);
        } else {
            switchableCamera.setActiveCamera(cameraNameFront);
        }
        isFrontCameraActive = !isFrontCameraActive;
    }

    public void setCamera(HardwareMap hardwareMap, String cameraName) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, cameraName);
        if (webcamName == cameraNameLeft) {
            isFrontCameraActive = false;
        } else {
            isFrontCameraActive = true;
        }
        this.switchableCamera.setActiveCamera(webcamName);
    }
}
