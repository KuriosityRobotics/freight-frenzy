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
 *
 * @author dkou
 */
public class LocalizationConsumer implements VuforiaConsumer {
    // Accessable values
    VuforiaLocalizer vuforia;
    Point robotCoordinatesWebcam;
    Orientation robotRotationWebcam;

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
        final float mmPerInch = 25.4f;
        final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constants for perimeter targets
        final float halfField = 72 * mmPerInch;
        final float quadField = 36 * mmPerInch;

        // Class Members
        OpenGLMatrix lastLocation = null;

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

        // Initialize Localization

        // Target Trackables
        // TODO: Change labelling to this year's
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        allTrackables.addAll(targetsUltimateGoal);

        // set Wall target locations
        // TODO: Change targets to this year
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // TODO: Edit camera positioning so it matches
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 270, 0));

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
}
