
package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.FULL_FIELD_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_FIELD_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.HALF_TILE_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.ONE_AND_HALF_TILE_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.ONE_TILE_MM;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanData;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final Point[] TARGETS = {
            // all in our coordinate system
            new Point(0, (ONE_AND_HALF_TILE_MM + ONE_TILE_MM)/25.4),
            new Point(ONE_AND_HALF_TILE_MM /25.4, FULL_FIELD_MM /25.4),
            new Point((FULL_FIELD_MM - ONE_AND_HALF_TILE_MM)/25.4, FULL_FIELD_MM /25.4),
            new Point(FULL_FIELD_MM /25.4, (ONE_AND_HALF_TILE_MM + ONE_TILE_MM)/25.4)
    };

    private final WebcamName cameraName;

    private VuforiaTrackables freightFrenzyTargets;

    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedData = null;
    private volatile Double detectedHorizPeripheralAngle = null;
    private volatile Double detectedVertPeripheralAngle = null;

    // change states here
    private final Servo rotator;
    private final DcMotor cameraEncoder;

    private double oldCameraAngle = 0.0;
    private double cameraAngle = 0.0;
    private double cameraAngleVelocity = 0.0;
    private static final double CAMERA_ENCODER_TO_RADIAN = 2.0 * Math.PI / 8192.0;
    private Point targetVuMark = new Point(0,0);

    private static final double ROTATOR_LEFT_POS = 0.15745917129498305;
    private static final double ROTATOR_RIGHT_POS = 0.8217672476721208;
    private static final double ROTATOR_CENTER_POS = 0.48961320987;
    private static final double ROTATOR_ANGLE_RANGE = Math.PI/4;

    private long lastUpdateTime = 0;

    private final LocationProvider locationProvider;
    private final PhysicalCamera physicalCamera;

    private final Robot robot;

    public VuforiaLocalizationConsumer(Robot robot, LocationProvider locationProvider, PhysicalCamera physicalCamera, WebcamName cameraName, HardwareMap hwMap) {
        this.robot = robot;
        this.locationProvider = locationProvider;
        this.physicalCamera = physicalCamera;
        this.cameraName = cameraName;
        rotator = hwMap.get(Servo.class, "webcamPivot");
        cameraEncoder = hwMap.get(DcMotor.class, "webcamPivot");
        rotator.setPosition(ROTATOR_CENTER_POS);

        resetEncoders();
    }

    @Override
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them, deactivate first because weird stuff can occur if we don't
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }

        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();

        // Identify the targets so vuforia can use them
        identifyTarget(0, "Blue Storage", -HALF_FIELD_MM, ONE_AND_HALF_TILE_MM, (float) Math.toRadians(90), (float) Math.toRadians(90));
        identifyTarget(1, "Blue Alliance Wall", HALF_TILE_MM, HALF_FIELD_MM, (float) Math.toRadians(90), 0f);
        identifyTarget(2, "Red Storage", -HALF_FIELD_MM, -ONE_AND_HALF_TILE_MM, (float) Math.toRadians(90), (float) Math.toRadians(90));
        identifyTarget(3, "Red Alliance Wall", HALF_TILE_MM, -HALF_FIELD_MM, (float) Math.toRadians(90), (float) Math.toRadians(180));
    }

    @Override
    public void update() {
        synchronized (this) {
            setCameraAngle(calculateOptimalCameraAngle());
            updateCameraAngleAndVelocity();

            long currentTimeMillis = SystemClock.elapsedRealtime();
            trackVuforiaTargets();

            RealMatrix data = getLocationRealMatrix();

            // hopefully this doesn't do bad thread stuff
            if (data != null) robot.sensorThread.addGoodie(new KalmanData(1, data), currentTimeMillis);
        }
    }

    /**
     * Chooses which VuMark for camera to face based on current robot position
     * @return target camera heading in radians
     */
    private double calculateOptimalCameraAngle(){
        double robotX = locationProvider.getPose().x;
        double robotY = locationProvider.getPose().y;
        double robotHeading = locationProvider.getPose().heading;

        double pivotX = robotX + SERVO_FORWARD_DISPLACEMENT_MM /25.4 * Math.sin(robotHeading) + SERVO_LEFT_DISPLACEMENT_MM /25.4 * Math.cos(robotHeading);
        double pivotY = robotY + SERVO_FORWARD_DISPLACEMENT_MM /25.4 * Math.cos(robotHeading) - SERVO_LEFT_DISPLACEMENT_MM /25.4 * Math.sin(robotHeading);

        Pose cameraPose = new Pose(pivotX, pivotY, robotHeading);
        ArrayList<Point> possibilities = new ArrayList<>();

        for (Point target : TARGETS){
            double relHeading = cameraPose.relativeHeadingToPoint(target);
            if (Math.abs(relHeading) < ROTATOR_ANGLE_RANGE){
                possibilities.add(target);
            }
        }

        if (possibilities.size() == 0) {
            targetVuMark = new Point(0,0);
            return ROTATOR_CENTER_POS;
        }

        targetVuMark = cameraPose.nearestPoint(possibilities);
        return cameraPose.relativeHeadingToPoint(targetVuMark);
    }

    private void setCameraAngle(double angle) {
        rotator.setPosition(angleToCameraPos(angle));
    }

    public double getCameraAngle() {
        return cameraAngle;
    }

    private void updateCameraAngleAndVelocity(){
        long currentUpdateTime = SystemClock.elapsedRealtime();
        double dTime = (currentUpdateTime - lastUpdateTime) / 1000.0;

        cameraAngle = -(double)cameraEncoder.getCurrentPosition() * CAMERA_ENCODER_TO_RADIAN;

        cameraAngleVelocity = (cameraAngle - oldCameraAngle) / dTime;

        oldCameraAngle = cameraAngle;
        lastUpdateTime = currentUpdateTime;
    }

    private void trackVuforiaTargets(){

        this.detectedData = null;
        this.detectedHorizPeripheralAngle = null;
        this.detectedVertPeripheralAngle = null;
        this.detectedTrackable = null;

        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cameraName, physicalCamera.translationMatrix().multiplied(physicalCamera.rotationMatrix()));
        }

        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {

                detectedTrackable = trackable;

                OpenGLMatrix robotLocationTransform = listener.getRobotLocation();
                OpenGLMatrix vuMarkPoseRelativeToCamera = listener.getFtcCameraFromTarget();

                if (robotLocationTransform != null) {
                    VectorF trans = vuMarkPoseRelativeToCamera.getTranslation();

                    this.detectedData = robotLocationTransform;
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    this.detectedHorizPeripheralAngle = angleWrap(Math.PI / 2 + Math.atan2(-tZ, tX));
                    this.detectedVertPeripheralAngle = angleWrap(Math.atan2(tZ, tY)-Math.PI/2);
                } else {
                    Log.d("Vision", "Cannot detect robot location although trackable is visible");
                }

                break;
            }
        }
    }

    private double angleToCameraPos(double a){
        return a * (ROTATOR_RIGHT_POS - ROTATOR_LEFT_POS) / (Math.PI) + ROTATOR_CENTER_POS;
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

            data.add("Camera Angle: " + Math.toDegrees(cameraAngle));
            data.add("Camera Angle Velocity: " + cameraAngleVelocity);
            data.add("Target VuMark x: " + targetVuMark.x + ", y: " + targetVuMark.y + ")");

            if (detectedTrackable == null) {
                data.add("No trackables detected");
            } else {
                data.add("Detected Trackable: " + detectedTrackable.getName());
                data.add("Horizontal Peripheral Angle: " + Math.toDegrees(detectedHorizPeripheralAngle));
                data.add("Vertical Peripheral Angle: " + Math.toDegrees(detectedVertPeripheralAngle));
                data.add("vufX: " + getLocationRealMatrix().getEntry(0,0));
                data.add("vufY: " + getLocationRealMatrix().getEntry(1,0));
                data.add("vufHeading: " + getLocationRealMatrix().getEntry(2,0));
            }

            return data;
        }
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float rx, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, RADIANS, rx, (float) 0.0, rz)));
    }

    public RealMatrix getLocationRealMatrix() {
        synchronized (this) {
            try {
                if (detectedData == null) {
                    return null;
                }

                if (detectedData == null) {
                    return null;
                }

                // filter out by peripherals
                if (Math.abs(detectedHorizPeripheralAngle) >= Math.toRadians(15) || Math.abs(detectedVertPeripheralAngle) >= Math.toRadians(10)){
                    return null;
                }

                // filter out by translational speed
                if (Math.hypot(locationProvider.getVelocity().x, locationProvider.getVelocity().y) > 2){
                    return null;
                }

                // filter out by angle speeds
                if (Math.abs(locationProvider.getVelocity().heading) > 0.15 || Math.abs(cameraAngleVelocity) > 0.05){
                    return null;
                }

                VectorF translation = detectedData.getTranslation();
                Point robotLocation = new Point(translation.get(0) / MM_PER_INCH, translation.get(1) / MM_PER_INCH);
                double heading = Orientation.getOrientation(detectedData, EXTRINSIC, XYZ, RADIANS).thirdAngle;

                // Convert from FTC coordinate system to ours
                double robotHeadingOurs = angleWrap(Math.PI - heading);
                double robotXOurs = robotLocation.y + (HALF_FIELD_MM / MM_PER_INCH);
                double robotYOurs = -robotLocation.x + (HALF_FIELD_MM / MM_PER_INCH);

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

    private void resetEncoders() {
        cameraEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}