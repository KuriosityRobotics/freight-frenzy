
package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VARIABLE_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VERTICAL_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_VERTICAL_DISPLACEMENT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.FULL_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_FIELD;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_TILE;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_TARGET_HEIGHT;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.ONE_AND_HALF_TILE;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.ONE_TILE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    private static final Point[] TARGETS = {
            // all in our coordinate system
            new Point(0, ONE_AND_HALF_TILE + ONE_TILE),
            new Point(ONE_AND_HALF_TILE, FULL_FIELD),
            new Point(FULL_FIELD - ONE_AND_HALF_TILE, FULL_FIELD),
            new Point(FULL_FIELD, ONE_AND_HALF_TILE + ONE_TILE)
    };

    private final WebcamName cameraName;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedData = null;
    private volatile Double detectedHorizPeripheralAngle = null;
    private volatile Double detectedVertPeripheralAngle = null;

    // change states here
    private final Servo rotator;

    private final Robot robot;

    public VuforiaLocalizationConsumer(Robot robot, WebcamName cameraName, HardwareMap hwMap) {
        this.robot = robot;
        this.cameraName = cameraName;
        rotator = hwMap.get(Servo.class, "webcamPivot");
        rotator.setPosition(TURRET_180);
    }

    private static final double TURRET_270 = .78988,
            TURRET_180 = .5,
            TURRET_90 = .0229;

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them, deactivate first because weird stuff can occur if we don't
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }

        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage", -HALF_FIELD, ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, (float) Math.toRadians(90), 0f, (float) Math.toRadians(90));
        identifyTarget(1, "Blue Alliance Wall", HALF_TILE, HALF_FIELD, MM_TARGET_HEIGHT, (float) Math.toRadians(90), 0f, 0f);
        identifyTarget(2, "Red Storage", -HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, (float) Math.toRadians(90), 0f, (float) Math.toRadians(90));
        identifyTarget(3, "Red Alliance Wall", HALF_TILE, -HALF_FIELD, MM_TARGET_HEIGHT, (float) Math.toRadians(90), 0f, (float) Math.toRadians(180));
    }

    // TODO: do rotate cam stuff
    private void setCameraAngle(double angle) {
        double rotationAngle = angle / (2 * Math.PI);
//        rotator.setPosition(ROTATOR_LEFT_POS + rotationAngle * (ROTATOR_RIGHT_POS - ROTATOR_LEFT_POS));
        //stub
    }

    private double getCamAngleTo(Point target) {
        Pose currentPosition = robot.sensorThread.getPose();
        double ret = Math.atan2(
                currentPosition.x - target.x,
                currentPosition.y - target.y
        ) - Math.PI - robot.sensorThread.getPose().heading;
        return ret;
    }

    @Override
    public void update() {
        synchronized (this) {
            this.detectedData = null;
            this.detectedHorizPeripheralAngle = null;
            this.detectedVertPeripheralAngle = null;
            this.detectedTrackable = null;

            // if a trackable isn't detected, there isn't a need to continue
            if (this.freightFrenzyTargets == null) {
                return;
            }

            // we do stuff
//            double cameraAngle = Arrays.stream(TARGETS).map(this::getCamAngleTo)
//                    .min(Comparator.naturalOrder())
//                    .orElse(0.);
//            setCameraAngle(cameraAngle);

            for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
                if (listener.isVisible()) {
                    detectedTrackable = trackable;

                    OpenGLMatrix cameraLoc = OpenGLMatrix
                            .translation(SERVO_FORWARD_DISPLACEMENT + CAMERA_VARIABLE_DISPLACEMENT, SERVO_LEFT_DISPLACEMENT, SERVO_VERTICAL_DISPLACEMENT + CAMERA_VERTICAL_DISPLACEMENT)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, RADIANS, (float) Math.PI / 2, (float) Math.toRadians(90), (float) Math.toRadians(33)));
                    listener.setCameraLocationOnRobot(cameraName, cameraLoc);

                    OpenGLMatrix robotLocationTransform = listener.getRobotLocation();
                    OpenGLMatrix vuMarkPoseRelativeToCamera = listener.getFtcCameraFromTarget();

                    Orientation rot = Orientation.getOrientation(vuMarkPoseRelativeToCamera, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    VectorF trans = vuMarkPoseRelativeToCamera.getTranslation();

                    if (robotLocationTransform != null && vuMarkPoseRelativeToCamera != null) {
                        this.detectedData = robotLocationTransform;

                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);
                        this.detectedHorizPeripheralAngle = angleWrap(Math.abs(Math.PI / 2 + Math.atan2(-tZ, tX)));
                        this.detectedVertPeripheralAngle = Math.abs(angleWrap(Math.atan2(tZ, tY))-Math.PI/2);
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
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }
    }

    /**
     * Get robot position messages via vuforia localization data
     *
     * @return Data for the Vuforia Localization and Telemetry Dump
     */
    public ArrayList<String> logPositionAndDetection() {
        synchronized (this) {
            ArrayList<String> data = new ArrayList<>();

            if (detectedTrackable == null) {
                data.add("No trackables detected");
            } else {
                data.add("Detected Trackable: " + detectedTrackable.getName());
                data.add("Horizontal Peripheral Angle: " + Math.toDegrees(detectedHorizPeripheralAngle));
                data.add("Vertical Peripheral Angle: " + Math.toDegrees(detectedVertPeripheralAngle));
            }

            return data;
        }
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, RADIANS, rx, ry, rz)));
    }

    public RealMatrix getLocationRealMatrix() {
        synchronized (this) {
            try {
                if (!robot.visionThread.getManagedCamera().vuforiaActive) {
                    return null;
                }

                if (detectedData == null) {
                    return null;
                }

                if (detectedHorizPeripheralAngle >= Math.toRadians(15)){
                    return null;
                }
                if (detectedVertPeripheralAngle >= Math.toRadians(10)){
                    return null;
                }
                if (Math.hypot(robot.sensorThread.getVelocity().x, robot.sensorThread.getVelocity().y) > 2){
                    return null;
                }
                if (Math.abs(robot.sensorThread.getVelocity().heading) > 0.2){
                    return null;
                }

                VectorF translation = detectedData.getTranslation();
                Point robotLocation = new Point(translation.get(0) / MM_PER_INCH, translation.get(1) / MM_PER_INCH);
                double heading = Orientation.getOrientation(detectedData, EXTRINSIC, XYZ, RADIANS).thirdAngle;

                // Convert from FTC coordinate system to ours
                double robotHeadingOurs = angleWrap(Math.PI - heading);
                double robotXOurs = robotLocation.y + (HALF_FIELD / MM_PER_INCH);
                double robotYOurs = -robotLocation.x + (HALF_FIELD / MM_PER_INCH);

                logValues(new Pose(robotLocation, heading), new Pose(robotXOurs, robotYOurs, robotHeadingOurs));

                return MatrixUtils.createRealMatrix(new double[][]{
                        {robotXOurs},
                        {robotYOurs},
                        {robotHeadingOurs}
                });
            } catch (Exception e) {
                return null;
            }
        }
    }

    // for debug
    private void logValues(Pose ftcLocation, Pose ourSystem) {
        // Fancy formatting :sunglas:

        Log.e("Vision", "FTC Coordinate System");
        Log.v("Vision", "FTC x: " + ftcLocation.x);
        Log.v("Vision", "FTC y: " + ftcLocation.y);
        Log.v("Vision", "FTC heading: " + Math.toDegrees(ftcLocation.heading));

        Log.e("Vision", "Our Coordinate System");
        Log.v("Vision", "Our x: " + ourSystem.x);
        Log.v("Vision", "Our y: " + ourSystem.y);
        Log.v("Vision", "Our heading: " + Math.toDegrees(ourSystem.heading));
    }
}
