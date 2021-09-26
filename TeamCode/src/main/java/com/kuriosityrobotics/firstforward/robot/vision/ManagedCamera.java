package com.kuriosityrobotics.firstforward.robot.vision;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaConsumer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

import de.esoco.coroutine.CoroutineScope;

public final class ManagedCamera {
    private static final String VUFORIA_LICENCE_KEY = "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
            "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
            "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
            "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
            "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
            "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
            "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";

    private  List<VuforiaConsumer> vuforiaConsumers;
    private OpenCvCamera openCvCamera;

    private static boolean vuforiaInitialisedYet;

    private List<OpenCvConsumer> openCvConsumers;

    public ManagedCamera(String cameraNameString, HardwareMap hardwareMap, VuforiaConsumer[] vuforiaConsumers, OpenCvConsumer... openCvConsumers) {
        var cameraName = hardwareMap.get(WebcamName.class, cameraNameString);

        if (vuforiaConsumers != null) {
            if(!vuforiaInitialisedYet) {
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
                parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                parameters.cameraName = cameraName;

                var vuforia = ClassFactory.getInstance().createVuforia(parameters);

                var vuforiaCoro = first(consume((VuforiaConsumer::update)));
                CoroutineScope.launch(scope ->
                {
                    if (vuforiaConsumers != null) {
                        for (int i = 0; i < vuforiaConsumers.length; i++) {
                            vuforiaCoro.runAsync(scope, vuforiaConsumers[i]);
                        }
                    }
                });

                vuforiaInitialisedYet = true;
                openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);
            } else {
                throw new RuntimeException("ManagedCamera(String, HardwareMap, VuforiaConsumer, ...) constructor called multiple times.  Running more than one instance of Vuforia isn't supported and will lead to a crash.");
            }
        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName);
        }

        openCvCamera.openCameraDeviceAsync(() -> {
            openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
            openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

            openCvCamera.setPipeline(new CameraConsumerProcessor());
            openCvCamera.startStreaming(1920, 1080);
        });

    }

    public ManagedCamera(String cameraName, HardwareMap hardwareMap, OpenCvConsumer... openCvConsumers) {
        this(cameraName, hardwareMap, null, openCvConsumers);
    }

    private final class CameraConsumerProcessor extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            //Log.e("ManagedCamera", String.format("processFrame() called at %d", SystemClock.currentThreadTimeMillis()));

            var vuforiaCoro = first(consume((VuforiaConsumer::update)));
            var openCvCoro = first(consume((OpenCvConsumer consumer) -> { //!!
                var matCopy = input.clone();
                consumer.processFrame(matCopy);
                matCopy.release(); // c++ moment
            }));

            CoroutineScope.launch(scope ->
            {
                vuforiaConsumers.forEach(consumer -> vuforiaCoro.runAsync(scope, consumer));
                openCvConsumers.forEach(consumer -> openCvCoro.runAsync(scope, consumer));
            });

            return input;
        }
    }
}
