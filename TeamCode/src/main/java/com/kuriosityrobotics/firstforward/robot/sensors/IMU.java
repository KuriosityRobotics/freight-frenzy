package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.rotate;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ojalgo.matrix.Primitive64Matrix;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class IMU implements Module {
    public static final String CALIBRATION_FILE = "imu_calibration.json";
    private static final File SETTINGS_FILE = AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE);

    private final BNO055IMU imu;
    private final ExtendedKalmanFilter filter;
    private final long timeOffset;

    private double offset;

    private long lastUpdateTime = 0;
    private double lastTheta = 0;
    private Acceleration acceleration;

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

    private double lastYTheta;

    public boolean isOffGround() {
        return abs(angleWrap(lastYTheta)) > toRadians(3);
//                ||         yAngVelos.stream().allMatch(n -> n > toRadians(20));
    }

    public void update() {
        acceleration = imu.getLinearAcceleration();

        var rawOrientation = imu.getAngularOrientation();
        lastYTheta = rawOrientation.toAxesReference(AxesReference.EXTRINSIC).toAxesOrder(AxesOrder.XYZ).secondAngle;

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
            lastTheta = angle;

            filter.builder()
                    .time(lastUpdateTime)
                    .mean(angle)
                    .variance(toRadians(.5))
                    .stateToOutput(Primitive64Matrix.FACTORY.row(0, 0, 1))
                    .correct();
        }

//        Log.d("IMU", String.valueOf(toDegrees(lastTheta)));
    }

    public void resetPose(Pose pose) {
        resetHeading(pose.heading);
    }

    public Point getAcceleration() {
        var accel = acceleration.toUnit(DistanceUnit.INCH);
        var point = Vector2D.of(accel.zAccel, accel.xAccel);
        point = rotate(point, -lastTheta);
        return new Point(point.getX(), point.getY());
    }

    private void resetHeading(double theta) {
        lastTheta = theta;
        this.offset = imu.getAngularOrientation().firstAngle + theta;
    }

    public double getAngle(Orientation orientation) {
        return offset - orientation.firstAngle;
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add("Heading:  " + toDegrees(angleWrap(lastTheta)));
            add("acceleration:  " + acceleration.toUnit(DistanceUnit.INCH));
            add("is off ground:  " + isOffGround());
            add("y ang:  " + toDegrees(lastYTheta));
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

    @Override
    public int maxFrequency() {
        return 20;
    }
}
