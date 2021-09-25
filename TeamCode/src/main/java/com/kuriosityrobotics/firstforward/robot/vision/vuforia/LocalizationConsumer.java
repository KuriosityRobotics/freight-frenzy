package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Trackable;

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
    private VuforiaTrackables ultimateGoalTargets;

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
        this.ultimateGoalTargets = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = ultimateGoalTargets.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = ultimateGoalTargets.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = ultimateGoalTargets.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = ultimateGoalTargets.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = ultimateGoalTargets.get(4);
        frontWallTarget.setName("Front Wall Target");

        this.ultimateGoalTargets.activate();

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // TODO: Edit camera positioning on the robot
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * MM_PER_INCH;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * MM_PER_INCH;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // Let all the trackable listeners know where the phone is.
        CameraName cameraName = vuforia.getCameraName();
        for (VuforiaTrackable trackable : ultimateGoalTargets) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setCameraLocationOnRobot(cameraName, cameraLocationOnRobot);
        }
    }

    /**
     * Update robot position if any trackable is visible
     */
    private void updatePosition() {
        boolean targetVisible = false;

        if (this.ultimateGoalTargets == null) {
            RobotLog.v("All trackables are null");
            return;
        }

        for (VuforiaTrackable trackable : this.ultimateGoalTargets) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                RobotLog.v("Visible Target", trackable.getName());
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

    private void updateDetection() {
        if (this.ultimateGoalTargets == null) {
            RobotLog.v("All trackables are null");
            return;
        }

        for (VuforiaTrackable trackable : this.ultimateGoalTargets) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                RobotLog.v("Target Visible", trackable.getName());
                RobotLog.v("Target Position", trackable.getLocation());

                // When your variable naming causes you to use weird variable names
                VectorF locationoftheTrackable = trackable.getLocation().getTranslation();
                this.trackableLocation = new Point(locationoftheTrackable.get(0) / MM_PER_INCH, locationoftheTrackable.get(1) / MM_PER_INCH);

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    this.lastLocation = robotLocationTransform;
                }
                break;
            }
        }
    }

    /**
     * Do nothing since this is for OpenCV and that's not what Vuforia is using
     */
    @Override
    public void update() {
//        ArrayList<String> robotData = logPosition();
//        for (String msg: robotData) {
//            RobotLog.v(msg);
//        }
    }

    /**
     * Remember to call when opmode finishes
     */
    public void deactivate() {
        this.ultimateGoalTargets.deactivate();
    }

    /**
     * Get robot position messages via vuforia localization data
     * @return
     */
    public ArrayList<String> logPositionandDetection() {
        updatePosition();
        updateDetection();

        ArrayList<String> data = new ArrayList<>();

        data.add("Robot Coordinates: " + (this.robotCoordinatesWebcam != null ?
                this.robotCoordinatesWebcam.toString() :
                "UKNOWN"));

        data.add("Robot Orientation: " + (this.robotRotationWebcam != null ?
                this.robotRotationWebcam.toString() :
                "UKNOWN"));

        data.add("Detected Trackable: " + (this.detectedTrackable != null ?
                this.detectedTrackable.toString() :
                "None found lol"));

        data.add("Trackable Location: " + (this.trackableLocation != null ?
                this.trackableLocation.toString() :
                "UNKNOWN"));

        return data;
    }
}