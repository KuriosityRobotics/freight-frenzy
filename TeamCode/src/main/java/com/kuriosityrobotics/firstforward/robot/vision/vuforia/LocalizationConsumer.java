package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.math.MathFunctions.angleWrap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.math.Point;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
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
public class LocalizationConsumer implements VuforiaConsumer {
    // Accessible values
    private static final float MM_PER_INCH = 25.4f;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedLocation = null;

    // Camera positions on robot (both front and back)
    // current pos matches tuning, not supposed to match actual pos on the robot
    // it is correct but vuforia sucks when it is too close to the wall target(2-3 inches off)
    private static final float CAMERA_LEFT_FORWARD_DISPLACEMENT = .125f * MM_PER_INCH;
    private static final float CAMERA_LEFT_VERTICAL_DISPLACEMENT = 7.25f * MM_PER_INCH;
    private static final float CAMERA_LEFT_LEFT_DISPLACEMENT = 5.5f * MM_PER_INCH;

    final float CAMERA_FRONT_FORWARD_DISPLACEMENT = 8.075f * MM_PER_INCH;   // eg: Enter the forward distance from the center of the robot to the camera lens
    final float CAMERA_FRONT_VERTICAL_DISPLACEMENT = 15.313f * MM_PER_INCH;   // eg: Camera is 6 Inches above ground
    final float CAMERA_FRONT_LEFT_DISPLACEMENT = 0.185f * MM_PER_INCH;   // eg: Enter the left distance from the center of the robot to the camera lens

    // Constants for perimeter targets
    private static final float MM_TARGET_HEIGHT = 6f * MM_PER_INCH;
    private static final float HALF_FIELD = 70.75f * MM_PER_INCH;
    private static final float HALF_TILE = 11.75f * MM_PER_INCH;
    private static final float ONE_AND_HALF_TILE = 35.25f * MM_PER_INCH;


    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them
        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage",       -HALF_FIELD,  ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  HALF_TILE,   HALF_FIELD,      MM_TARGET_HEIGHT, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   HALF_TILE,  -HALF_FIELD,      MM_TARGET_HEIGHT, 90, 0, 180);

        OpenGLMatrix cameraLeftLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_LEFT_FORWARD_DISPLACEMENT, CAMERA_LEFT_LEFT_DISPLACEMENT, CAMERA_LEFT_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 180, 0));

        OpenGLMatrix cameraFrontLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FRONT_FORWARD_DISPLACEMENT, CAMERA_FRONT_LEFT_DISPLACEMENT, CAMERA_FRONT_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 30));

        // Let all the trackable listeners know where the phone is.
        SwitchableCamera switchableCamera = (SwitchableCamera) vuforia.getCamera();
        // cameraNames[0] is Webcam 1
        // cameraNames[1] is Webcam 2
        CameraName[] cameraNames = switchableCamera.getMembers();
        for (VuforiaTrackable trackable : freightFrenzyTargets) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setCameraLocationOnRobot(cameraNames[0], cameraLeftLocationOnRobot);
            listener.setCameraLocationOnRobot(cameraNames[1], cameraFrontLocationOnRobot);
        }
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
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    detectedTrackable = trackable;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();
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
            double robotHeadingFIRST = Orientation.getOrientation(detectedLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
            Log.v("Vision", "FTC heading: " + robotHeadingFIRST);

            // convert to our coordinate system because FTC coordinate system bad and our is better and more readable :sunglas: :lemonthink:
            double robotHeadingOurs = (Math.toDegrees(angleWrap(Math.toRadians(-robotHeadingFIRST))));
            double robotXOurs = (robotLocation.y + (HALF_FIELD / MM_PER_INCH)) - (CAMERA_FRONT_FORWARD_DISPLACEMENT * Math.cos(robotHeadingOurs) / MM_PER_INCH) - (CAMERA_FRONT_LEFT_DISPLACEMENT * Math.sin(robotHeadingOurs) / MM_PER_INCH);
            double robotYOurs = (-robotLocation.x + (HALF_FIELD / MM_PER_INCH)) + (CAMERA_FRONT_FORWARD_DISPLACEMENT * Math.sin(robotHeadingOurs) / MM_PER_INCH) - (CAMERA_FRONT_LEFT_DISPLACEMENT * Math.cos(robotHeadingOurs) / MM_PER_INCH);

            Log.v("Vision", "x: " + robotXOurs);
            Log.v("Vision", "y: " + robotYOurs);
            Log.v("Vision", "heading: " + robotHeadingOurs);

            return MatrixUtils.createRealMatrix(new double[][]{
                    {robotXOurs, 0},
                    {robotYOurs, 0},
                    {robotHeadingOurs, 0}
            });
        }
    }
}