package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.*;

import com.kuriosityrobotics.firstforward.robot.math.Point;

import org.apache.commons.math3.linear.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;

/**
 * Defining a Vuforia localization consumer
 */
public class LocalizationConsumer implements VuforiaConsumer {
    // Accessible values
    private VuforiaTrackables freightFrenzyTargets;

    private VuforiaTrackable detectedTrackable;
    private OpenGLMatrix detectedLocation = null;
    private CameraName cameraName;

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them
        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage",       -HALF_FIELD, ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", HALF_TILE, HALF_FIELD, MM_TARGET_HEIGHT, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", HALF_TILE,  -HALF_FIELD, MM_TARGET_HEIGHT, 90, 0, 180);

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, -90, -90, 0));

        // Let all the trackable listeners know where the phone is.
        cameraName = vuforia.getCameraName();
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