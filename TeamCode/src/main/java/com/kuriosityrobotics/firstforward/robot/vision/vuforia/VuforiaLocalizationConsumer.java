package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum.DatumType.CORRECTION;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.HALF_FIELD_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.HALF_TILE_MEAT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.TARGET_HEIGHT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.TILE_MEAT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.TILE_TAB_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VARIABLE_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.CAMERA_VERTICAL_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_FORWARD_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_LEFT_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.SERVO_VERTICAL_DISPLACEMENT_MM;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
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
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

/**
 * Defining a Vuforia localization consumer
 */

// This is for a single webcam(not a switchable cam)
public class VuforiaLocalizationConsumer implements VuforiaConsumer {
    private static final double CAMERA_ENCODER_TO_RADIAN = 2.0 * PI / 8192.0;
    private static final double ROTATOR_CENTER_POS = .295;
    private static final double ROTATOR_BACK_POS = .96;
    private static final double ROTATOR_ANGLE_RANGE = 3 * PI / 2;
    private static final Map<Target, Point> offsets = new HashMap<>() {{
        put(Target.BLUE_STORAGE, new Point(0, 0));
        put(Target.BLUE_ALLIANCE_WALL, new Point(0, 0));
        put(Target.RED_STORAGE, new Point(0, 0));
        put(Target.RED_ALLIANCE_WALL, new Point(0, 0));
    }};
    private final WebcamName cameraName;
    // change states here
    private final Servo rotator;
    private final DcMotor cameraEncoder;
    private final LocationProvider locationProvider;
    private final PhysicalCamera physicalCamera;
    private final Robot robot;
    public boolean doneCalibrating = false;
    boolean cameraEncoderSetYet = false;
    private VuforiaTrackables freightFrenzyTargets;
    private volatile VuforiaTrackable detectedTrackable = null;
    private volatile OpenGLMatrix detectedData = null;
    private volatile Double detectedHorizPeripheralAngle = null;
    private volatile Double detectedVertPeripheralAngle = null;
    private double oldCameraAngle = 0.0;
    private double cameraAngle = 0.0;
    private double cameraAngleVelocity = 0.0;
    private Point targetVuMark = new Point(0, 0);
    private long lastUpdateTime = 0;
    private Long startTime = null;
    private double cameraAngleOffset = 0;
    private long lastAcceptedTime = 0;
    private long lastDetectedTime = 0;

    public VuforiaLocalizationConsumer(Robot robot, LocationProvider locationProvider, PhysicalCamera physicalCamera, WebcamName cameraName, HardwareMap hwMap) {
        this.locationProvider = locationProvider;
        this.physicalCamera = physicalCamera;
        this.cameraName = cameraName;
        this.robot = robot;
        rotator = hwMap.get(Servo.class, "webcamPivot");
        cameraEncoder = hwMap.get(DcMotor.class, "webcamPivot");

        setCameraAngle(0);

        startTime = SystemClock.elapsedRealtime();
    }

    private static Point ftcToOurs(Point point) {
        return new Point(
                (point.y + HALF_FIELD_MM) / MM_PER_INCH,
                (-point.x + HALF_FIELD_MM) / MM_PER_INCH
        );
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
        identifyTargets();
    }

    private void validateOffsets() {
        if (!offsets.keySet().equals(new HashSet<>(Arrays.asList(Target.values()))))
            throw new Error("Offsets for all targets should be initialised.");
    }

    private synchronized void identifyTargets() {
        validateOffsets();

        for (var entry : offsets.entrySet()) {
            Target target = entry.getKey();
            Point offset = entry.getValue();

            identifyTarget(
                    target.index,
                    target.name(),
                    (float) (target.dx - (offset.y * MM_PER_INCH)),
                    (float) (target.dy + (offset.x * MM_PER_INCH)),
                    target.dz,
                    target.rx,
                    target.ry,
                    target.rz
            );
        }
    }

    private synchronized void addOffset(Target target, Point delta) {
        Point currentOffset = offsets.get(target);
        if (currentOffset == null)
            throw new AssertionError("Offset shouldn't be null.");

        Point newOffset = currentOffset.add(delta);

        offsets.put(target, newOffset);
        identifyTargets();
    }

    public void changeAllianceWallOffsetBy(Point delta) {
        if (Robot.isBlue)
            addOffset(Target.BLUE_ALLIANCE_WALL, delta);
        else
            addOffset(Target.RED_ALLIANCE_WALL, delta);
    }

    private Point getOffset(Target target) {
        return offsets.get(target);
    }

    @Override
    public void update() {
        synchronized (this) {
            if (SystemClock.elapsedRealtime() >= startTime + 500) {
                if (!cameraEncoderSetYet) {
                    resetEncoders(0);
                    cameraEncoderSetYet = true;
                }
            }

            if (cameraEncoderSetYet) {
                if (robot.started() || !robot.isAuto()) {
                    setCameraAngle(0);
//                    setCameraAngle(calculateOptimalCameraAngle());
                } else if (!Robot.isCarousel && !doneCalibrating) {
                    setCameraAngle(0);
                } else {
                    setCameraAngle(PI);
                }

                updateCameraAngleAndVelocity();

                trackVuforiaTargets();

                long fetchTime = SystemClock.elapsedRealtime();
                RealMatrix data = getLocationRealMatrix();

                // hopefully this doesn't do bad thread stuff
                if (data != null) {
                    robot.sensorThread.addGoodie(new KalmanDatum(CORRECTION, data), fetchTime);
                    lastAcceptedTime = SystemClock.elapsedRealtime();
                    Log.v("KF", "adding vuf goodie, passed filters");
                }
            }
        }
    }

    /**
     * Chooses which VuMark for camera to face based on current robot position
     *
     * @return target camera heading in radians
     */
    private double calculateOptimalCameraAngle() {
        double robotX = locationProvider.getPose().x;
        double robotY = locationProvider.getPose().y;
        double robotHeading = locationProvider.getPose().heading;

        double pivotX = robotX + SERVO_FORWARD_DISPLACEMENT_MM / MM_PER_INCH * Math.sin(robotHeading) + SERVO_LEFT_DISPLACEMENT_MM / MM_PER_INCH * Math.cos(robotHeading);
        double pivotY = robotY + SERVO_FORWARD_DISPLACEMENT_MM / MM_PER_INCH * Math.cos(robotHeading) - SERVO_LEFT_DISPLACEMENT_MM / MM_PER_INCH * Math.sin(robotHeading);

        Pose cameraPose = new Pose(pivotX, pivotY, robotHeading);
        ArrayList<Point> possibilities = new ArrayList<>();

        for (Target target : Target.values()) {
            Point offset = getOffset(target);
            Point ourPoint = ftcToOurs(
                    new Point(target.dx, target.dy)
            ).add(offset);

            double relHeading = cameraPose.relativeHeadingToPoint(ourPoint);
            if (Math.abs(relHeading) < ROTATOR_ANGLE_RANGE)
                possibilities.add(ourPoint);
        }

        if (possibilities.isEmpty()) {
            targetVuMark = new Point(0, 0);
            return 0;
        }

        targetVuMark = cameraPose.nearestPoint(possibilities);

        double relativeHeading = cameraPose.relativeHeadingToPoint(targetVuMark);

        if (
                relativeHeading > PI / 2 || // otherwise we hit the outtake
                        relativeHeading < -PI / 4 // otherwise we hit the cables
        )
            return 0;

        return relativeHeading;
    }

    public double getTargetCameraAngle() {
        return cameraAngle;
    }

    private void setCameraAngle(double angle) {
        rotator.setPosition(angleToCameraPos(angle));
    }

    private void updateCameraAngleAndVelocity() {
        long currentUpdateTime = SystemClock.elapsedRealtime();
        double dTime = (currentUpdateTime - lastUpdateTime) / 1000.0;

        cameraAngle = -(double) (cameraEncoder.getCurrentPosition()) * CAMERA_ENCODER_TO_RADIAN;
        cameraAngle -= cameraAngleOffset;

        cameraAngleVelocity = (cameraAngle - oldCameraAngle) / dTime;

        oldCameraAngle = cameraAngle;
        lastUpdateTime = currentUpdateTime;
    }

    private void trackVuforiaTargets() {

//        this.detectedData = null;
//        this.detectedHorizPeripheralAngle = null;
//        this.detectedVertPeripheralAngle = null;
//        this.detectedTrackable = null;

        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            OpenGLMatrix cameraLoc = OpenGLMatrix
                    .translation(SERVO_FORWARD_DISPLACEMENT_MM + CAMERA_VARIABLE_DISPLACEMENT_MM * (float) Math.cos(cameraAngle), SERVO_LEFT_DISPLACEMENT_MM - CAMERA_VARIABLE_DISPLACEMENT_MM * (float) Math.sin(cameraAngle), SERVO_VERTICAL_DISPLACEMENT_MM + CAMERA_VERTICAL_DISPLACEMENT_MM)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, RADIANS, (float) (PI / 2 + Math.toRadians(33)), (float) (PI / 2 - cameraAngle), 0));
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cameraName, cameraLoc);
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cameraName, physicalCamera.translationMatrix().multiplied(physicalCamera.rotationMatrix()));
        }

        boolean sawAny = false;
        for (VuforiaTrackable trackable : this.freightFrenzyTargets) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {
                sawAny = true;

                detectedTrackable = trackable;
                lastDetectedTime = SystemClock.elapsedRealtime();

                OpenGLMatrix robotLocationTransform = listener.getRobotLocation();
                OpenGLMatrix vuMarkPoseRelativeToCamera = listener.getFtcCameraFromTarget();
                if (robotLocationTransform != null) {

                    Log.v("KF", "vuf saw");

                    VectorF trans = vuMarkPoseRelativeToCamera.getTranslation();

                    this.detectedData = robotLocationTransform;
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
//                    Log.v("KF", "tX: " + tX);
//                    Log.v("KF", "tY: " + tY);
//                    Log.v("KF", "tZ: " + tZ);

                    this.detectedHorizPeripheralAngle = angleWrap(PI / 2 + Math.atan2(-tZ, tX));
                    this.detectedVertPeripheralAngle = angleWrap(Math.atan2(tZ, tY) - PI / 2);
                } else {
                    Log.e("Vision", "Cannot detect robot location although trackable is visible");
                }

                break;
            }
        }

        if (!sawAny) {
            this.detectedData = null;
            this.detectedHorizPeripheralAngle = null;
            this.detectedVertPeripheralAngle = null;
            this.detectedTrackable = null;
        }
    }

    private double angleToCameraPos(double a) {
        return (a * (ROTATOR_BACK_POS - ROTATOR_CENTER_POS)) / PI + ROTATOR_CENTER_POS;
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

                RealMatrix robotLocation = getLocationRealMatrix();

                if (robotLocation != null) {
                    data.add("vufX: " + robotLocation.getEntry(0, 0));
                    data.add("vufY: " + robotLocation.getEntry(1, 0));
                    data.add("vufHeading: " + robotLocation.getEntry(2, 0));
                } else {
                    data.add("not using vuforia goodies rn");
                }
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
/*
                {
                    var trackableLocation = new Vector3D(0, 5.75, 58);
                    var centre = physicalCamera.pinholeCamera().getLocationOnFrame(trackableLocation);

                    if (centre.distance(new Vector2D(800 / 2., 448/2.)) > 150) {
                        Log.v("kf", "DISCARD by projection, " + centre + " " + centre.distance(new Vector2D(800 / 2., 448/2.)));
                        return null;
                    }
                }*/
                // filter out by peripherals
                if (Math.abs(detectedHorizPeripheralAngle) >= Math.toRadians(28) || Math.abs(detectedVertPeripheralAngle) >= Math.toRadians(25)) {
                    Log.v("kf", "DISCARD by perif, " + Math.abs(Math.toDegrees(detectedHorizPeripheralAngle)) + ", " + Math.abs(Math.toDegrees(detectedVertPeripheralAngle)));
                    return null;
                }

                // filter out by translational speed
                if (Math.hypot(robot.sensorThread.getOdometryVelocity().x, robot.sensorThread.getOdometryVelocity().y) > 1) {
                    Log.v("kf", "DISCARD by trans vel, " + Math.hypot(robot.sensorThread.getOdometryVelocity().x, robot.sensorThread.getOdometryVelocity().y));
                    return null;
                }

                // filter out by angle speeds
                if (Math.abs(robot.sensorThread.getOdometryVelocity().heading) > 0.02 || Math.abs(cameraAngleVelocity) > 0.05) {
                    Log.v("kf", "DISCARD by heading vel, " + Math.abs(robot.sensorThread.getOdometryVelocity().heading) + ", " + Math.abs(cameraAngleVelocity));
                    return null;
                }

                VectorF translation = detectedData.getTranslation();
                Pose ourSystem = dataToOurSystem(translation);

                return MatrixUtils.createRealMatrix(new double[][]{
                        {ourSystem.x},
                        {ourSystem.y},
                        {ourSystem.heading}
                });

            } catch (Exception e) {
                return null;
            }
        }
    }

    private Pose dataToOurSystem(VectorF dataTranslation) {
        Point robotLocation = new Point(dataTranslation.get(0) / MM_PER_INCH, dataTranslation.get(1) / MM_PER_INCH);
        double heading = Orientation.getOrientation(detectedData, EXTRINSIC, XYZ, RADIANS).thirdAngle;

        // Convert from FTC coordinate system to ours
        double robotHeadingOurs = angleWrap(PI - heading);
        double robotXOurs = robotLocation.y + (HALF_FIELD_MM / MM_PER_INCH);
        double robotYOurs = -robotLocation.x + (HALF_FIELD_MM / MM_PER_INCH);

        return new Pose(robotXOurs, robotYOurs, robotHeadingOurs);
    }

    public Pose lastRawRobotPosition() {
        if (detectedData == null) {
            return null;
        }
        return dataToOurSystem(detectedData.getTranslation());
    }

    private void resetEncoders(double currentAngle) {
        cameraAngleOffset = currentAngle;
        cameraEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public long getLastAcceptedTime() {
        return lastAcceptedTime;
    }

    public long getLastDetectedTime() {
        return lastDetectedTime;
    }

    enum Target {
        BLUE_STORAGE(0, -HALF_FIELD_MM,
                1.5f * TILE_MEAT_MM + 1.5f * TILE_TAB_MM,
                TARGET_HEIGHT_MM,
                (float) Math.toRadians(90), 0f, (float) Math.toRadians(90)),
        BLUE_ALLIANCE_WALL(1,
                0.5f * TILE_TAB_MM + HALF_TILE_MEAT_MM,
                HALF_FIELD_MM,
                TARGET_HEIGHT_MM,
                (float) Math.toRadians(90), 0f, 0f),
        RED_STORAGE(2,
                -HALF_FIELD_MM,
                -(1.5f * TILE_MEAT_MM + 1.5f * TILE_TAB_MM),
                TARGET_HEIGHT_MM,
                (float) Math.toRadians(90), 0f, (float) Math.toRadians(90)),
        RED_ALLIANCE_WALL(3, 0.5f * TILE_TAB_MM + HALF_TILE_MEAT_MM,
                -HALF_FIELD_MM,
                TARGET_HEIGHT_MM,
                (float) Math.toRadians(90), 0f, (float) Math.toRadians(180)
        );

        public final int index;
        public final float dx, dy, dz, rx, ry, rz;

        Target(int index, float dx, float dy, float dz, float rx, float ry, float rz) {
            this.index = index;
            this.dx = dx;
            this.dy = dy;
            this.dz = dz;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
        }
    }
}