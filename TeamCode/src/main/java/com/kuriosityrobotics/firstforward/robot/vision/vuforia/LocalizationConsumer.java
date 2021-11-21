package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.kuriosityrobotics.firstforward.robot.math.Point;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
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

    private VuforiaTrackable detectedTrackable;
    private OpenGLMatrix detectedLocation = null;

    // current pos matches tuning, not supposed to match actual pos on the robot
    private static final float CAMERA_FORWARD_DISPLACEMENT = 5.375f * MM_PER_INCH;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 2.5f * MM_PER_INCH;
    private static final float CAMERA_LEFT_DISPLACEMENT = 3.0f * MM_PER_INCH;

    // Constants for perimeter targets
    private static final float mmTargetHeight = 6 * MM_PER_INCH;
    private static final float halfField        = 72 * MM_PER_INCH;
    private static final float halfTile         = 12 * MM_PER_INCH;
    private static final float oneAndHalfTile   = 36 * MM_PER_INCH;

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them
        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, -90, -90, 0));

        // Let all the trackable listeners know where the phone is.
        CameraName cameraName = vuforia.getCameraName();
        for (VuforiaTrackable trackable : freightFrenzyTargets) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setCameraLocationOnRobot(cameraName, cameraLocationOnRobot);
        }
    }

    @Override
    public void update() {
        boolean targetVisible = false;

        // if a trackable isn't detected, there isn't a need to continue
        if (this.freightFrenzyTargets == null) {
            return;
        }

        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                detectedTrackable = trackable;
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    this.detectedLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (!targetVisible) {
            this.detectedLocation = null;
            this.detectedTrackable = null;
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
        ArrayList<String> data = new ArrayList<>();

        if (detectedLocation == null) {
            data.add("No trackables detected");
            return data;
        }
        else {
            data.add("Detected Trackable: " + detectedTrackable.getName());
        }

        return data;
    }

    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public RealMatrix getFormattedMatrix() {
        if (detectedLocation == null) {
            return null;
        }

        VectorF translation = detectedLocation.getTranslation();
        Point robotLocation = new Point(Math.round(translation.get(0) / MM_PER_INCH), Math.round(translation.get(1) / MM_PER_INCH));
        double robotHeading = Orientation.getOrientation(detectedLocation, EXTRINSIC, XYZ, RADIANS).thirdAngle;

        return MatrixUtils.createRealMatrix(new double[][]{
                {robotLocation.x, 0},
                {robotLocation.y, 0},
                {robotHeading, 0}
        });
    }
}