
package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_LEFT_FORWARD_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_LEFT_LEFT_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_LEFT_VERTICAL_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_TILE;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_TARGET_HEIGHT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.ONE_AND_HALF_TILE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.vision.SingleManagedCamera;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;

/**
 * Defining a Vuforia localization consumer
 */

// This is for a single webcam(not a switchable cam)
public class VuforiaLocalizationConsumer implements VuforiaConsumer {
    private final WebcamName cameraName;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedLocation = null;

    // change states here
    public static CameraOrientation cameraOrientation = CameraOrientation.CENTER;
    private final Servo rotator;

    public VuforiaLocalizationConsumer(WebcamName cameraName, HardwareMap hwMap) {
        this.cameraName = cameraName;
        rotator = hwMap.get(Servo.class, "WebcamRotator");
    }

    // TODO: tune
    private static final double ROTATOR_LEFT_POS = 0;
    private static final double ROTATOR_CENTER_POS = 0;
    private static final double ROTATOR_RIGHT_POS = 0;

    public enum CameraOrientation {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them, deactivate first because weird stuff
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

            // we do stuff
            int camRotation;
            if (cameraOrientation.equals(CameraOrientation.LEFT)) {
                rotator.setPosition(ROTATOR_LEFT_POS);
                camRotation = 180;
            } else if (cameraOrientation.equals(CameraOrientation.CENTER)) {
                rotator.setPosition(ROTATOR_CENTER_POS);
                camRotation = 90;
            } else {
                rotator.setPosition(ROTATOR_RIGHT_POS);
                camRotation = 0;
            }

            for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
                if (listener.isVisible()) {
                    detectedTrackable = trackable;

                    OpenGLMatrix cameraLoc = OpenGLMatrix
                            .translation(CAMERA_LEFT_FORWARD_DISPLACEMENT, CAMERA_LEFT_LEFT_DISPLACEMENT, CAMERA_LEFT_VERTICAL_DISPLACEMENT)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, camRotation, 0));
                    listener.setCameraLocationOnRobot(cameraName, cameraLoc);

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
        cameraOrientation = CameraOrientation.CENTER;
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

    public RealMatrix getVuforiaMatrix() {
        synchronized (this) {
            if (SingleManagedCamera.vuforiaActive) {
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
                    {robotXOurs},
                    {robotYOurs},
                    {robotHeadingOurs}
            });
        }
    }
}
