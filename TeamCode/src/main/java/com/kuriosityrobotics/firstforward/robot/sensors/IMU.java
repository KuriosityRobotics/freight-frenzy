package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum.DatumType.CORRECTION;
import static com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum.DatumType.PREDICTION;
import static com.kuriosityrobotics.firstforward.robot.sensors.LocalizeKalmanFilter.diagonal;
import static java.lang.Double.NaN;
import static java.lang.Math.pow;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter.KalmanDatum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.math3.linear.MatrixUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.nio.file.Files;

public class IMU {
    private static final String CALIBRATION_FILE = "imu_calibration.json";

    private final BNO055IMU imu;
    private LocalizeKalmanFilter filter;
    private double offset;
    private final long timeOffset;

    void setKalmanFilter(LocalizeKalmanFilter filter) {
        this.filter = filter;
    }

    public IMU(HardwareMap hardwareMap, LocalizeKalmanFilter filter) {
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.filter = filter;

        var params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        timeOffset = SystemClock.elapsedRealtimeNanos() - System.nanoTime();

        if (AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE).exists())
            loadCalibration();
        else {
            Log.w("IMU", "Waiting for calibration...");
            try {
                var thread = new Thread(() -> {
                    while (!imu.isMagnetometerCalibrated() || !imu.isGyroCalibrated());
                    saveCalibration();
                    Log.w("IMU", "Finished calibration");
                });
                thread.start();
                thread.join(Robot.DEBUG ? 0 : 1000);
            } catch (InterruptedException e) {
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
        var orientation = imu.getAngularOrientation();
        orientation.acquisitionTime += timeOffset;

        if (orientation.acquisitionTime != lastUpdateTime && orientation.acquisitionTime < lastUpdateTime) // paranoia
            if (Robot.DEBUG)
                throw new Error("Overflow");
            else
                new Error("Overflow").printStackTrace();

        if (orientation.acquisitionTime > lastUpdateTime) {
            lastUpdateTime = orientation.acquisitionTime;
            filter.addGoodie(new KalmanDatum(CORRECTION, MatrixUtils.createColumnRealMatrix(
                    new double[]{NaN, NaN, getAngle(orientation)}
            ), diagonal(.01, .01, toRadians(3)), 2), lastUpdateTime / 1_000_000);
        }

        Log.d("IMU", String.valueOf(toDegrees(getAngle(imu.getAngularOrientation()))));
    }

    public void resetHeading(double theta) {
        this.offset = imu.getAngularOrientation().firstAngle + theta;
    }

    public double getAngle(Orientation orientation) {
        return offset - orientation.firstAngle;
    }
}
