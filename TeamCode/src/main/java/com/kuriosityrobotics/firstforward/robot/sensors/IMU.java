package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ojalgo.matrix.Primitive64Matrix;

import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class IMU implements Telemeter {
    public static final String CALIBRATION_FILE = "imu_calibration.json";
    private static final File SETTINGS_FILE = AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE);
    private static final double BIAS_PER_REVOLUTION = toRadians(-1);
    private final BNO055IMU imu;
    private final ExtendedKalmanFilter filter;
    private final ExtendedKalmanFilter offsetFilter = new ExtendedKalmanFilter(new double[1], toRadians(9));
    private final long timeOffset;
    private final double variance = toRadians(4);
    private double offset;
    private long lastUpdateTime = 0;
    private double lastTheta = 0;
    private double totalRevolutions = 0;

    public IMU(HardwareMap hardwareMap, ExtendedKalmanFilter filter) {
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.filter = filter;

        var params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        timeOffset = ((SystemClock.elapsedRealtime() * 1_000_000) - System.nanoTime()) / 1_000_000;
        if (AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE).exists())
            loadCalibration();
        else {
            Log.w("IMU", "Waiting for calibration...");
            try {
                CompletableFuture.runAsync(() -> {
                    while (!imu.isMagnetometerCalibrated() || !imu.isGyroCalibrated()) ;
                    saveCalibration();
                    Log.w("IMU", "Finished calibration.");
                }).get(Robot.DEBUG ? 0 : 1000, TimeUnit.MILLISECONDS);
            } catch (InterruptedException | TimeoutException | ExecutionException e) {
                Log.w("IMU", "Calibration failed");
                e.printStackTrace();
            }
        }

    }

    private void loadCalibration() {
        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(SETTINGS_FILE)));
    }

    public void saveCalibration() {
        ReadWriteFile.writeFile(SETTINGS_FILE, imu.readCalibrationData().serialize());
        Log.w("IMU", "Saved calibration data to file");
    }

    public BNO055IMU.CalibrationData readCalibrationData() {
        return imu.readCalibrationData();
    }

    int i = 0;
    public void update() {
        if ((i = (i + 1) % 25) != 0)
            return;

        var rawOrientation = imu.getAngularOrientation();
        rawOrientation.acquisitionTime /= 1_000_000;
        rawOrientation.acquisitionTime += timeOffset;

        if (rawOrientation.acquisitionTime != lastUpdateTime && rawOrientation.acquisitionTime != 0 && rawOrientation.acquisitionTime < lastUpdateTime) // paranoia
            if (Robot.DEBUG)
                throw new Error("Overflow");
            else
                new Error("Overflow").printStackTrace();


        if (rawOrientation.acquisitionTime > lastUpdateTime) {
            lastUpdateTime = rawOrientation.acquisitionTime;
            var angle = getAngle(rawOrientation);

            // if it's more than pi/2 between two measurements, assume it's wrapped around
//            if (abs(angle - lastTheta) < PI / 2) {
//                totalRevolutions += (angle - lastTheta) / (2 * PI);
//                Log.d("IMU", "Difference:  " + ((angle - lastTheta) / (2 * PI)));
//                Log.d("IMU", "total:  " + totalRevolutions);
//            } else {
//                Log.d("IMU", "wrapped");
//            }
            lastTheta = angle;

            filter.datumBuilder()
                    .time(lastUpdateTime)
                    .mean(angle + (totalRevolutions * BIAS_PER_REVOLUTION))
                    .variance(toRadians(2))
                    .stateToOutput(Primitive64Matrix.FACTORY.row(0, 0, 1))
                    .correct();
        }

//        Log.d("IMU", String.valueOf(toDegrees(lastTheta)));
    }

    public void resetPose(Pose pose) {
        resetHeading(pose.heading);
    }

    private void resetHeading(double theta) {
        lastTheta = theta;
        this.offset = imu.getAngularOrientation().firstAngle + theta;
    }

    public double getAngle(Orientation orientation) {
        return offset - orientation.firstAngle;
    }

    @Override
    public Iterable<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("Heading:  " + toDegrees(angleWrap(lastTheta)));
        }};
    }

    @Override
    public String getName() {
        return "IMU";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}
