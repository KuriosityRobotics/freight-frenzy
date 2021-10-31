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

import java.util.Arrays;
import java.util.List;

import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;

public final class ManagedCamera {
    private static final String VUFORIA_LICENCE_KEY = "Aa7As2v/////AAABmdo32M3mD0L7gNHJd7lGgBkeZ3HRMWih" +
            "/BXVOQ30fS92DoZsqpNfKE7K0dGGEyKsB3ZzaGQTNs6TBVhvpr8ov2LA3D3ME5leS/51WKkau0ms9RWwVyyl9sDyPl" +
            "rEdw2wm0zG26lzzYCtTod6ESKz/5em2Vmav2tZnT7rBsBDjBlcEbOz6VyAqzyzbFWNiMNIu/p93XhAHtNQ4jVHACz3" +
            "uBMoVQrrntKXjxWB5dadGn5tASHDERG7/D82foRTsII9VSmH1sltVLtFy2asD/jpJEtYmxlZdB9KNhdiaAA99Y7Ckd" +
            "mE/ZC4PC0yd3KiDO+j1F9UIQBI0oQXAhKjocmv4jmbMXGmzCtvASJnC8/WcesL";

    private VuforiaConsumer vuforiaConsumer;
    private OpenCvCamera openCvCamera;

    private boolean vuforiaInitialisedYet;

    private List<OpenCvConsumer> openCvConsumers;

    public ManagedCamera(String cameraNameString, HardwareMap hardwareMap, VuforiaConsumer vuforiaConsumer, OpenCvConsumer... openCvConsumers) {
        this.vuforiaConsumer = vuforiaConsumer;
        this.openCvConsumers = Arrays.asList(openCvConsumers);
        var cameraName = hardwareMap.get(WebcamName.class, cameraNameString);

        if (vuforiaConsumer != null) {
            if(!vuforiaInitialisedYet) {
                // setup vuforia
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
                parameters.vuforiaLicenseKey = VUFORIA_LICENCE_KEY;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                parameters.cameraName = cameraName;

                VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
                vuforiaConsumer.setup(vuforia);

                vuforiaInitialisedYet = true;
                openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters);
            } else {
                // control hub does not like multiple vuforias, so don't try spawning more than 1
                throw new RuntimeException("ManagedCamera(String, HardwareMap, VuforiaConsumer, ...) constructor called multiple times.  Running more than one instance of Vuforia isn't supported and will lead to a crash.");
            }
        } else {
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(cameraName);
        }

        // set stuff up so opencv can also run
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
                vuforiaCoro.runAsync(scope, vuforiaConsumer);
                openCvConsumers.forEach(consumer -> openCvCoro.runAsync(scope, consumer));
            });

            return input;
        }
    }
}
