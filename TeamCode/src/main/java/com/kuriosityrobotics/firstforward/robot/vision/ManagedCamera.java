package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

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

    public WebcamName cameraName1;
    public WebcamName cameraName2;

    public WebcamName activeCamera;
    private VuforiaLocalizer vuforia;

    public ManagedCamera(WebcamName name1, WebcamName name2, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);
        this.cameraName1 = name1;
        this.cameraName2 = name2;

        activateCamera(cameraName1);
    }

    public void activateCamera(WebcamName cameraName) {
        if (activeCamera == cameraName) {
            return;
        }

        if (vuforia != null) {
            vuforia.close();
            vuforia = null;
        }
        if (vuforiaConsumer != null) {
            // setup vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraName = cameraName;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            vuforiaConsumer.setup(vuforia);
            openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);
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

        this.activeCamera = cameraName;
    }

//    public ManagedCamera(String camera1, String camera2, HardwareMap hardwareMap, OpenCvConsumer... openCvConsumers) {
//        this(camera1, camera2, hardwareMap, null, openCvConsumers);
//    }

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
