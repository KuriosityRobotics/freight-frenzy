package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Defining a Vuforia localization consumer
 */
public class LocalizationConsumer implements VuforiaConsumer {
    // Accessable values
    VuforiaLocalizer vuforia;
    Point robotCoordinatesWebcam;
    Orientation robotRotationWebcam;
    OpenGLMatrix lastLocation = null;
    VuforiaTrackables wallTargets;

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;

        // Constants for perimeter targets
        final float mmPerInch = 25.4f;
        final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
        final float halfField        = 72 * mmPerInch;
        final float halfTile         = 12 * mmPerInch;
        final float oneAndHalfTile   = 36 * mmPerInch;

        // Class Members

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

        // Initialize Localization

        wallTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // Get trackables
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        // TODO: Edit camera positioning on the robot
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vuforia.getCameraName(), cameraLocationOnRobot);
        }

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                RobotLog.v("Visible Target", trackable.getName());

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    VectorF translation = lastLocation.getTranslation();
                    this.robotCoordinatesWebcam = new Point(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
                    this.robotRotationWebcam = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                }

                RobotLog.v("Pos (in): ", this.robotCoordinatesWebcam);
                RobotLog.v("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotRotationWebcam.firstAngle, robotRotationWebcam.secondAngle, robotRotationWebcam.thirdAngle);
            }
        }
    }

    @Override
    public void update() {
        RobotLog.v("Position", robotCoordinatesWebcam.toString());
        RobotLog.v("Rotation", robotRotationWebcam.toString());
    }

    public Point getPosition() {
        return this.robotCoordinatesWebcam;
    }

    public Orientation getRobotOrientation() {
        return this.robotRotationWebcam;
    }
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = wallTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
