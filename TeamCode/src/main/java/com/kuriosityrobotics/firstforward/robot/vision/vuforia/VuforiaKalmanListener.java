package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.Robot.assertThat;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static java.text.MessageFormat.format;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;
import java.util.List;

public class VuforiaKalmanListener extends VuforiaTrackableDefaultListener implements Telemeter {
    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;
    private final LocationProvider locationProvider;
    private final ExtendedKalmanFilter filter;
    private VuMarkDetection lastDetection;

    public VuforiaKalmanListener(
            VuforiaTrackable trackable,
            VuforiaLocalizationConsumer vuforiaLocalizationConsumer,
            LocationProvider locationProvider,
            ExtendedKalmanFilter filter
    ) {
        super(trackable);
        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;
        this.locationProvider = locationProvider;
        this.filter = filter;
    }

    @SuppressWarnings("ConstantConditions")
    public synchronized void onTracked(TrackableResult trackableResult, CameraName cameraName, Camera camera, VuforiaTrackable child) {
        super.onTracked(trackableResult, cameraName, camera, child);
        assertThat(isVisible());

        setCameraLocationOnRobot(cameraName, vuforiaLocalizationConsumer.getCameraLocationOnRobot());

        OpenGLMatrix vuMarkPoseRelativeToCamera = getFtcCameraFromTarget();
        assertThat(vuMarkPoseRelativeToCamera != null);

        VectorF trans = vuMarkPoseRelativeToCamera.getTranslation();

        double tX = trans.get(0);
        double tY = trans.get(1);
        double tZ = trans.get(2);
//                    Log.v("KF", "tX: " + tX);
//                    Log.v("KF", "tY: " + tY);
//                    Log.v("KF", "tZ: " + tZ);

        lastDetection = new VuMarkDetection(
                trackable,
                getRobotLocation(),
                angleWrap(PI / 2 + Math.atan2(-tZ, tX)),
                angleWrap(Math.atan2(tZ, tY) - PI / 2),
                (long) (trackableResult.getTimeStamp() * 1000.)
        );
        var pose = lastDetection.getKuriosityPose();
        vuforiaLocalizationConsumer.lastVuforiaPosition = pose;

        if (isValidDetection(lastDetection)) {
            vuforiaLocalizationConsumer.lastAcceptedTime = lastDetection.getDetectedTime();
            filter.datumBuilder()
                    .mean(pose.x, pose.y, pose.heading)
                    .variance(.04, .04, toRadians(3 * 3))
                    .correct();
        }
    }

    @Override
    public synchronized void onNotTracked() {
        super.onNotTracked();
        assertThat(!isVisible());
        this.lastDetection = null;
    }

    private synchronized boolean isValidDetection(VuMarkDetection detection) {
        // filter out by peripherals
        if (Math.abs(detection.getDetectedHorizPeripheralAngle()) >= toRadians(28)
                || Math.abs(detection.getDetectedVertPeripheralAngle()) >= toRadians(25)) {
            Log.v("kf", format("DISCARD by perif, {0} deg x, {1} deg y", toDegrees(detection.getDetectedHorizPeripheralAngle()), toDegrees(detection.getDetectedVertPeripheralAngle())));
            return false;
        }

        // filter out by translational speed
        if (locationProvider.getOrthVelocity() > 1) {
            Log.v("kf", format("DISCARD by trans vel, {0} in/s", locationProvider.getOrthVelocity()));
            return false;
        }

        // filter out by angle speeds
        if (locationProvider.getVelocity().heading > 0.02 || Math.abs(vuforiaLocalizationConsumer.getCameraAngleVelocity()) > 0.05) {
            Log.v("kf", format("DISCARD by heading vel, {0} deg, {1} deg/s", toDegrees(locationProvider.getVelocity().heading), toDegrees(vuforiaLocalizationConsumer.getCameraAngleVelocity())));
            return false;
        }
        return true;
    }

    @Override
    public synchronized List<String> getTelemetryData() {
        assertThat(isVisible());

        ArrayList<String> data = new ArrayList<>();

        if (lastDetection == null)
            return data;

        data.add("Horizontal Peripheral Angle: " + toDegrees(lastDetection.getDetectedHorizPeripheralAngle()));
        data.add("Vertical Peripheral Angle: " + toDegrees(lastDetection.getDetectedVertPeripheralAngle()));

        var robotLocation = lastDetection.getKuriosityPose();

        data.add("vufPose: " + robotLocation);
        data.add("isValidDetection:  " + isValidDetection(lastDetection));


        return data;
    }

    @Override
    public String getName() {
        return trackable.getName();
    }

    @Override
    public boolean isOn() {
        return isVisible();
    }
}
