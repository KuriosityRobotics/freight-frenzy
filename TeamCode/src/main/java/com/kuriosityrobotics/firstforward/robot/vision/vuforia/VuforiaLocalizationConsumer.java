
package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.math.Point;

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

import java.util.ArrayList;

/**
 * Defining a Vuforia localization consumer
 */
public class VuforiaLocalizationConsumer implements VuforiaConsumer {
    // Accessible values
    private static final float MM_PER_INCH = 25.4f;

    private WebcamName cameraName1;
    private WebcamName cameraName2;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedLocation = null;

    // Camera positions on robot (both front and left)
    // it is correct but vuforia sucks when it is too close to the wall target(2-3 inches off)
    private static final float CAMERA_LEFT_FORWARD_DISPLACEMENT = .125f * MM_PER_INCH;
    private static final float CAMERA_LEFT_VERTICAL_DISPLACEMENT = 7.25f * MM_PER_INCH;
    private static final float CAMERA_LEFT_LEFT_DISPLACEMENT = 5.5f * MM_PER_INCH;

    private static final float CAMERA_FRONT_FORWARD_DISPLACEMENT = 8.075f * MM_PER_INCH;
    private static final float CAMERA_FRONT_VERTICAL_DISPLACEMENT = 15.313f * MM_PER_INCH;
    private static final float CAMERA_FRONT_LEFT_DISPLACEMENT = 0.185f * MM_PER_INCH;

    // Constants for perimeter targets
    private static final float MM_TARGET_HEIGHT = 6f * MM_PER_INCH;
    private static final float HALF_FIELD = 70f * MM_PER_INCH;
    private static final float ONE_TILE = 23.5f * MM_PER_INCH;
    private static final float FULL_FIELD = HALF_FIELD * 2f;
    private static final float ONE_AND_HALF_TILE = ONE_TILE * 1.5f;
    private static final float HALF_TILE = ONE_TILE * 0.5f;

    private static final double HALF_ROBOT_WIDTH = 11.75 / 2;
    private static final double HALF_ROBOT_LENGTH = 12.75 / 2;

    private static final OpenGLMatrix cameraLeftLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_LEFT_FORWARD_DISPLACEMENT, CAMERA_LEFT_LEFT_DISPLACEMENT, CAMERA_LEFT_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 180, 0));

    private static final OpenGLMatrix cameraFrontLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FRONT_FORWARD_DISPLACEMENT, CAMERA_FRONT_LEFT_DISPLACEMENT, CAMERA_FRONT_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 30));

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
                        listener.setCameraLocationOnRobot(switchableCamera.getCameraName(), cameraLeftLocationOnRobot);
                    } else if (switchableCamera.getActiveCamera() == cameraName2) {
                        listener.setCameraLocationOnRobot(switchableCamera.getCameraName(), cameraFrontLocationOnRobot);
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

    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public RealMatrix getFormattedMatrix() {
        synchronized (this) {
            if (detectedLocation == null) {
                return null;
            }

            VectorF translation = detectedLocation.getTranslation();
            Point robotLocation = new Point(Math.round(translation.get(0) / MM_PER_INCH), Math.round(translation.get(1) / MM_PER_INCH));
            double heading = Orientation.getOrientation(detectedLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;

            // Convert from FTC coordinate system to ours
            double robotHeadingOurs = Math.toDegrees(angleWrap(Math.toRadians(180 - heading)));
            double robotXOurs = robotLocation.y + (HALF_FIELD / MM_PER_INCH);
            double robotYOurs = -robotLocation.x + (HALF_FIELD / MM_PER_INCH);

            // Fancy formatting :sunglas
//            Log.e("Vision", "FTC Coordinate System");
//            Log.v("Vision", "FTC x: " + robotLocation.x);
//            Log.v("Vision", "FTC y: " + robotLocation.y);
//            Log.v("Vision", "FTC heading: " + heading);
//
//            Log.e("Vision", "Our Coordinate System");
//            Log.v("Vision", "Our x: " + robotXOurs);
//            Log.v("Vision", "Our y: " + robotYOurs);
//            Log.v("Vision", "Our heading: " + robotHeadingOurs);

            return MatrixUtils.createRealMatrix(new double[][]{
                    {robotXOurs, 0},
                    {robotYOurs, 0},
                    {Math.toRadians(robotHeadingOurs), 0}
            });
        }
    }
}
