
package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import android.util.Log;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.MatrixUtil;
import com.kuriosityrobotics.firstforward.robot.vision.ManagedCamera;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Mat;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.*;
import java.util.ArrayList;

/**
 * Defining a Vuforia localization consumer
 */
public class VuforiaLocalizationConsumer implements VuforiaConsumer {
    private WebcamName cameraName1;
    private WebcamName cameraName2;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedLocation = null;

    private SwitchableCamera switchableCamera;

    public VuforiaLocalizationConsumer(WebcamName cameraName1, WebcamName cameraName2) {
        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
    }

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        this.switchableCamera = (SwitchableCamera) vuforia.getCamera();

        // Get trackables & activate them
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }

        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage",       -HALF_FIELD,  ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  HALF_TILE,   HALF_FIELD,      MM_TARGET_HEIGHT, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   HALF_TILE,  -HALF_FIELD,      MM_TARGET_HEIGHT, 90, 0, 180);
    }

    @Override
    public void update() {
        synchronized (this) {
            this.detectedLocation = null;
            this.detectedTrackable = null;

            // if a trackable isn't detected, there isn't a need to continue
            if (this.freightFrenzyTargets == null) {
                return;
            }

            for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
                if (listener.isVisible()) {
                    detectedTrackable = trackable;
                    // The listener does not understand that we have coordinates for the switchable cameras
                    // It thinks we're at (0,0,0)
                    // so we correct it here :sunglas:
                    if (switchableCamera.getActiveCamera() == cameraName1) {
                        listener.setCameraLocationOnRobot(switchableCamera.getCameraName(), CAMERA_LEFT_LOCATION_ON_ROBOT);
                    } else if (switchableCamera.getActiveCamera() == cameraName2) {
                        listener.setCameraLocationOnRobot(switchableCamera.getCameraName(), CAMERA_FRONT_LOCATION_ON_ROBOT);
                    }

                    OpenGLMatrix robotLocationTransform = listener.getRobotLocation();
                    if (robotLocationTransform != null) {
                        this.detectedLocation = robotLocationTransform;
                    } else {
                        Log.d("Vision", "Cannot detect robot location although trackable is visible");
                    }

                    break;
                }
            }
        }
    }

    /**
     * Remember to call when opmode finishes
     */
    public void deactivate() {
        this.freightFrenzyTargets.deactivate();
    }

    /**
     * Get robot position messages via vuforia localization data
     * @return
     * Data for the Vuforia Localization and Telemetry Dump
     */
    public ArrayList<String> logPositionandDetection() {
        synchronized (this) {
            ArrayList<String> data = new ArrayList<>();

            if (detectedTrackable == null) {
                data.add("No trackables detected");
            } else {
                data.add("Detected Trackable: " + detectedTrackable.getName());
            }

            return data;
        }
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public RealMatrix getFormattedMatrix() {
        synchronized (this) {
            if (!ManagedCamera.vuforiaActive) {
                return null;
            }

            if (detectedLocation == null) {
                return null;
            }

            VectorF translation = detectedLocation.getTranslation();
            Point robotLocation = new Point(translation.get(0) / MM_PER_INCH, translation.get(1) / MM_PER_INCH);
            double heading = Orientation.getOrientation(detectedLocation, EXTRINSIC, XYZ, RADIANS).thirdAngle;

            // Convert from FTC coordinate system to ours
            double robotHeadingOurs = angleWrap(Math.PI - heading);
            double robotXOurs = robotLocation.y + (HALF_FIELD / MM_PER_INCH);
            double robotYOurs = -robotLocation.x + (HALF_FIELD / MM_PER_INCH);

            // Fancy formatting :sunglas
//            Log.e("Vision", "FTC Coordinate System");
//            Log.v("Vision", "FTC x: " + robotLocation.x);
//            Log.v("Vision", "FTC y: " + robotLocation.y);
//            Log.v("Vision", "FTC heading: " + Math.toDegrees(heading));
//
//            Log.v("Vision", "Our Coordinate System");
//            Log.v("Vision", "Our x: " + robotXOurs);
//            Log.v("Vision", "Our y: " + robotYOurs);
//            Log.v("Vision", "Our heading: " + Math.toDegrees(robotHeadingOurs));

            return MatrixUtils.createRealMatrix(new double[][]{
                    {robotXOurs, 0},
                    {robotYOurs, 0},
                    {robotHeadingOurs, 0}
            });
        }
    }
}
