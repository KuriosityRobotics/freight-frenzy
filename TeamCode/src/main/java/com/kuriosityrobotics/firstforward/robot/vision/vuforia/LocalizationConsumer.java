package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Trackable;

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
    // Accessable values
    private static final float MM_PER_INCH = 25.4f;

    private Point robotCoordinatesWebcam;
    private Orientation robotRotationWebcam;
    private OpenGLMatrix lastLocation = null;
    private VuforiaTrackables freightFrenzyTargets;

    public Trackable detectedTrackable;
    public Point trackableLocation;

    @Override
    public void setup(VuforiaLocalizer vuforia) {

        // Constants for perimeter targets
        final float mmTargetHeight = (6) * MM_PER_INCH;          // the height of the center of the target image above the floor
        final float halfField        = 72 * MM_PER_INCH;
        final float halfTile         = 12 * MM_PER_INCH;
        final float oneAndHalfTile   = 36 * MM_PER_INCH;
        final float quadField  = 36 * MM_PER_INCH;

        // Get trackables
        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");

        this.freightFrenzyTargets.activate();

        // Identify the targets
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        // TODO: Edit camera positioning on the robot
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * MM_PER_INCH;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * MM_PER_INCH;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

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

        if (this.freightFrenzyTargets == null) {
            RobotLog.v("All trackables are null");
            return;
        }

        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                VectorF locationoftheTrackable = trackable.getLocation().getTranslation();
                this.trackableLocation = new Point(locationoftheTrackable.get(0) / MM_PER_INCH, locationoftheTrackable.get(1) / MM_PER_INCH);

                RobotLog.v("Visible Target", trackable.getName());
                RobotLog.v("Target Position", trackable.getLocation());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    this.lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            this.robotCoordinatesWebcam = new Point(translation.get(0) / MM_PER_INCH, translation.get(1) / MM_PER_INCH);
            this.robotRotationWebcam = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            RobotLog.v("Pos (in): ", this.robotCoordinatesWebcam);
            RobotLog.v("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotRotationWebcam.firstAngle, robotRotationWebcam.secondAngle, robotRotationWebcam.thirdAngle);
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
     */
    public ArrayList<String> logPositionandDetection() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Robot Coordinates: " + (this.robotCoordinatesWebcam != null ?
                this.robotCoordinatesWebcam.toString() :
                "UKNOWN"));

        data.add("Robot Orientation: " + (this.robotRotationWebcam != null ?
                this.robotRotationWebcam.toString() :
                "UKNOWN"));

        data.add("Detected Trackable: " + (this.detectedTrackable != null ?
                this.detectedTrackable.toString() :
                "None found"));

        data.add("Trackable Location: " + (this.trackableLocation != null ?
                this.trackableLocation.toString() :
                "UNKNOWN"));

        return data;
    }

    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public double getTargetFaceDirection() {
        if (this.detectedTrackable == null) {
            return 0;
        }

        double direction = 0;
        VuforiaTrackable foundTarget = freightFrenzyTargets.get(this.detectedTrackable.getId());

        if (foundTarget == null) {
            return direction;
        }
        else if (foundTarget.getName().equals("Blue Storage") || foundTarget.getName().equals("Red Storage")) {
            direction = 180;
        }
        else if (foundTarget.getName().equals("Blue Alliance Wall")) {
            direction = 270;
        }
        else if (foundTarget.getName().equals("Red Alliance Wall")) {
            direction = 90;
        }

        return direction;
    }

    public Point getTrackableCoordinates() {
        return this.trackableLocation;
    }

    public Point getCoordinates() {
        return this.robotCoordinatesWebcam;
    }

    public double getRobotRotation() {
        if (this.robotRotationWebcam == null) {
            this.robotRotationWebcam = new Orientation();
        }
        return this.robotRotationWebcam.secondAngle;
    }

    public RealMatrix getFormattedMatrix() {
        double robotX;
        double robotY;
        double robotRotation;

        double trackableLocationX;
        double trackableLocationY;
        double trackableAngle;

        robotX = (getCoordinates() != null ?
                getCoordinates().x:
                0);
        robotY = (getCoordinates() != null ?
                getCoordinates().y:
                0);
        robotRotation = (Math.abs(getRobotRotation() - 0.0) >= 0.01  ?
                getTargetFaceDirection():
                0);

        trackableLocationX = (getTrackableCoordinates() != null ?
                getTrackableCoordinates().x:
                0);
        trackableLocationY = (getTrackableCoordinates() != null ?
                getTrackableCoordinates().y:
                0);
        trackableAngle = (Math.abs(getTargetFaceDirection() - 0.0) >= 0.01  ?
                getTargetFaceDirection():
                0);

        RealMatrix obs = MatrixUtils.createRealMatrix(new double[][]{
                {trackableLocationX, robotX},
                {trackableLocationY, robotY},
                {trackableAngle, robotRotation}
        });

        return obs;
    }
}