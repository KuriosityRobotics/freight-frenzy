package com.kuriosityrobotics.firstforward.robot.sensors;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.KalmanData;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.time.Instant;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class IMU {
    private static final String CALIBRATION_FILE = "imu_calibration.json";

    private final BNO055IMU imu;
    private final ExtendedKalmanFilter filter;
    private double offset;
    private final long timeOffset;

    public IMU(HardwareMap hardwareMap, ExtendedKalmanFilter filter) {
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.filter = filter;

        var params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        timeOffset = Instant.now().toEpochMilli() - System.currentTimeMillis();

        if (AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE).exists())
            loadCalibration();
        else {
            Log.w("IMU", "Waiting for calibration...");
            try {
                CompletableFuture.runAsync(() -> {
                    while (!imu.isMagnetometerCalibrated() || !imu.isGyroCalibrated());
                    saveCalibration();
                    Log.w("IMU", "Finished calibration");
                }).get(Robot.DEBUG ? 0 : 1000, TimeUnit.MILLISECONDS);
            } catch (InterruptedException | TimeoutException | ExecutionException e) {
                Log.w("IMU", "Calibration failed");
                e.printStackTrace();
            }
        }

    }

    private void loadCalibration() {
        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE))));
    }

    private void saveCalibration() {
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE), imu.readCalibrationData().serialize());
    }

    private long lastUpdateTime = 0;
    public void update() {
        var rawOrientation = imu.getAngularOrientation();
        rawOrientation.acquisitionTime /= 1_000_000;
        rawOrientation.acquisitionTime += timeOffset;

        if (rawOrientation.acquisitionTime != lastUpdateTime && rawOrientation.acquisitionTime < lastUpdateTime) // paranoia
            if (Robot.DEBUG)
                throw new Error("Overflow");
            else
                new Error("Overflow").printStackTrace();


        if (rawOrientation.acquisitionTime > lastUpdateTime) {
            Log.d("IMU", imu.getVelocity().toString());

            lastUpdateTime = rawOrientation.acquisitionTime;
            filter.correction(KalmanData.gyroDatum(Instant.ofEpochMilli(lastUpdateTime), getAngle(rawOrientation)));
        }

//        Log.d("IMU", String.valueOf(toDegrees(getAngle(imu.getAngularOrientation()))));
    }

    public void resetPose(Pose pose) {
        imu.startAccelerationIntegration(new Position(DistanceUnit.INCH, pose.x, pose.y, 0, System.nanoTime()), new Velocity(), 10);
        resetHeading(pose.heading);
    }

    private void resetHeading(double theta) {
        this.offset = imu.getAngularOrientation().firstAngle + theta;
    }

    public double getAngle(Orientation orientation) {
        return offset - orientation.firstAngle;
    }
}
