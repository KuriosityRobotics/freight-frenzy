package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

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
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.Timer;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
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
public class VuforiaLocalizationConsumer {
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

    private final Servo rotator;
    private final DcMotor cameraEncoder;
    private final LocationProvider locationProvider;
    private final Robot robot;
    private final Timer cameraResetTimer;
    private final ExtendedKalmanFilter kalmanFilter;
    private boolean doneCalibrating = false;
    private VuforiaTrackables freightFrenzyTargets;
    private long lastUpdateTime = 0;
    private double cameraAngleOffset = 0;
    private double oldCameraAngle = 0.0;
    private double cameraAngle = 0.0;
    private double cameraAngleVelocity = 0.0;

    Pose lastVuforiaPosition;
    long lastAcceptedTime;
    long lastDetectedTime;

    public VuforiaLocalizationConsumer(
            Robot robot,
            ExtendedKalmanFilter kalmanFilter,
            LocationProvider locationProvider,
            HardwareMap hwMap
    ) {
        this.kalmanFilter = kalmanFilter;
        this.locationProvider = locationProvider;
        this.robot = robot;
        rotator = hwMap.get(Servo.class, "webcamPivot");
        cameraEncoder = hwMap.get(DcMotor.class, "intake");

        var resetAngle = calculateDesiredCameraAngle();
        setCameraAngle(resetAngle);
        cameraResetTimer = Timer.doIn(500, () -> resetEncoders(resetAngle));
    }

    private static Point ftcToOurs(Point point) {
        return new Point(
                (point.y + HALF_FIELD_MM) / MM_PER_INCH,
                (-point.x + HALF_FIELD_MM) / MM_PER_INCH
        );
    }

    private static double angleToCameraPos(double a) {
        return (a * (ROTATOR_BACK_POS - ROTATOR_CENTER_POS)) / PI + ROTATOR_CENTER_POS;
    }

    /**
         * Set up the Vuforia
     */
    public void setup(VuforiaLocalizer vuforia) {
        // Get trackables & activate them, deactivate first because weird stuff can occur if we don't
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }

        this.freightFrenzyTargets = vuforia.loadTrackablesFromAsset("FreightFrenzy");
        this.freightFrenzyTargets.activate();
        freightFrenzyTargets.forEach(trackable -> {
            var listener = new VuforiaKalmanListener(trackable, this, locationProvider, kalmanFilter);
            robot.getTelemetryDump().registerTelemeter(listener);
            trackable.setListener(listener);
        });

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
        if (Robot.isBlue())
            addOffset(Target.BLUE_ALLIANCE_WALL, delta);
        else
            addOffset(Target.RED_ALLIANCE_WALL, delta);
    }

    private Point getOffset(Target target) {
        return offsets.get(target);
    }

    /**
         * Update position
         */
    public void update() {
        synchronized (this) {
            if (cameraResetTimer.tick()) {
                setCameraAngle(calculateDesiredCameraAngle());

                updateCameraAngleAndVelocity();
            }
        }
    }

    /**
     * Chooses which VuMark for camera to face based on current robot position
     *
     * @return target camera heading in radians
     */
    private double calculateDesiredCameraAngle() {
        if (robot.started() || robot.isTeleOp()) {
            return angleToBestVuforiaTarget();
        } else if (!Robot.isCarousel() && !isDoneCalibrating())
            return 0;
        else
            return PI;
    }

    private double angleToBestVuforiaTarget() {
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
            return 0;
        }

        var targetVuMark = cameraPose.nearestPoint(possibilities);

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

    public OpenGLMatrix getCameraLocationOnRobot() {
        return OpenGLMatrix
                .translation(SERVO_FORWARD_DISPLACEMENT_MM + CAMERA_VARIABLE_DISPLACEMENT_MM * (float) Math.cos(cameraAngle), SERVO_LEFT_DISPLACEMENT_MM - CAMERA_VARIABLE_DISPLACEMENT_MM * (float) Math.sin(cameraAngle), SERVO_VERTICAL_DISPLACEMENT_MM + CAMERA_VERTICAL_DISPLACEMENT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, RADIANS, (float) (PI / 2 + toRadians(33)), (float) (PI / 2 - cameraAngle), 0));
    }

    /**
     * Remember to call when opmode finishes
     */
    public void deactivate() {
        if (this.freightFrenzyTargets != null) {
            this.freightFrenzyTargets.deactivate();
        }
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = this.freightFrenzyTargets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, RADIANS, rx, ry, rz)));
    }

    private void resetEncoders(double currentAngle) {
        cameraAngleOffset = currentAngle;
        cameraEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isDoneCalibrating() {
        return doneCalibrating;
    }

    public void setDoneCalibrating(boolean doneCalibrating) {
        this.doneCalibrating = doneCalibrating;
    }

    public double getCameraAngleVelocity() {
        return cameraAngleVelocity;
    }

    public Pose getLastVuforiaPosition() {
        return lastVuforiaPosition;
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
                (float) toRadians(90), 0f, (float) toRadians(90)),
        BLUE_ALLIANCE_WALL(1,
                0.5f * TILE_TAB_MM + HALF_TILE_MEAT_MM,
                HALF_FIELD_MM,
                TARGET_HEIGHT_MM,
                (float) toRadians(90), 0f, 0f),
        RED_STORAGE(2,
                -HALF_FIELD_MM,
                -(1.5f * TILE_MEAT_MM + 1.5f * TILE_TAB_MM),
                TARGET_HEIGHT_MM,
                (float) toRadians(90), 0f, (float) toRadians(90)),
        RED_ALLIANCE_WALL(3, 0.5f * TILE_TAB_MM + HALF_TILE_MEAT_MM,
                -HALF_FIELD_MM,
                TARGET_HEIGHT_MM,
                (float) toRadians(90), 0f, (float) toRadians(180)
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