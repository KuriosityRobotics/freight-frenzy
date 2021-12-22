package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;
import de.esoco.coroutine.*;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.opencv.core.Mat;
import static org.openftc.easyopencv.OpenCvCamera.ViewportRenderer.GPU_ACCELERATED;
import static org.openftc.easyopencv.OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW;
import org.openftc.easyopencv.*;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;

import java.util.Arrays;
import java.util.List;

public final class ManagedCamera {
    private static final String VUFORIA_LICENCE_KEY = "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
            "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
            "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
            "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
            "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
            "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
            "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";

    private VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;

    private static boolean vuforiaInitialisedYet;

    private List<OpenCvConsumer> openCvConsumers;

    public ManagedCamera(String cameraNameString, HardwareMap hardwareMap, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);
        WebcamName cameraName = hardwareMap.get(WebcamName.class, cameraNameString);

        if (vuforiaConsumer != null) {
            if(!vuforiaInitialisedYet) {
                // setup vuforia
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
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
            }
        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName);
        }

        // set stuff up so opencv can also run
        openCvCamera.openCameraDeviceAsync(new AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.setViewportRenderer(GPU_ACCELERATED);
                openCvCamera.setViewportRenderingPolicy(OPTIMIZE_VIEW);

                openCvCamera.setPipeline(new CameraConsumerProcessor());
                openCvCamera.startStreaming(1920, 1080);
            }

            @Override
            public void onError(int errorCode) {
                Log.d("ManagedCamera", "error: " + errorCode);
            }
        });

    }

    public ManagedCamera(String cameraName, HardwareMap hardwareMap, OpenCvConsumer... openCvConsumers) {
        this(cameraName, hardwareMap, null, openCvConsumers);
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
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

            return input;
        }
    }

    public void onClose() {
        this.vuforiaInitialisedYet = false;
    }
}
