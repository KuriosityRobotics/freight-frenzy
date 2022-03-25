package com.kuriosityrobotics.firstforward.robot.sensors;

import static java.util.concurrent.CompletableFuture.runAsync;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.KalmanDatum;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.HashMap;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable, Telemeter {

    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));
    /**
     * This is a singleton.  Pose position and history is persisted through this.
     * Remember to register it as a telemeter each time a new Robot is created.
     */
    private static final ExtendedKalmanFilter theKalmanFilter;

    static {
        theKalmanFilter = new ExtendedKalmanFilter(0, 0, 0);
    }

    private final Robot robot;
    private final Odometry odometry;
    private final IMU imu;
    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;

        this.odometry = new Odometry(robot.hardwareMap, getPose(), theKalmanFilter);
        this.imu = new IMU(robot.hardwareMap, theKalmanFilter);

        robot.telemetryDump.registerTelemeter(this);
        robot.telemetryDump.registerTelemeter(theKalmanFilter);
        robot.telemetryDump.registerTelemeter(odometry);
        robot.telemetryDump.registerTelemeter(imu);
    }

    public ExtendedKalmanFilter getKalmanFilter() {
        return theKalmanFilter;
    }

    public void resetPose(Pose pose) {
        getOdometry().setPose(pose);

        robot.telemetryDump.removeTelemeter(theKalmanFilter);

        theKalmanFilter.reset(pose.x, pose.y, pose.heading);
        getImu().resetPose(pose);

        robot.telemetryDump.registerTelemeter(theKalmanFilter);
    }

    @Override
    public void run() {
        var imuFuture = runAsync(imu::update);
        var expansionHubFuture = runAsync(robot.expansionHub::getBulkData);
        while (robot.running()) {
            if (imuFuture.isDone())
                imuFuture = runAsync(imu::update);

            if (expansionHubFuture.isDone())
                expansionHubFuture = runAsync(robot.expansionHub::getBulkData);

            robot.controlHub.getBulkData();
            odometry.update();

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.telemetryDump.sendPose(getPose().toDegrees());
                lastPoseSendTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    public void predict(KalmanDatum datum) {
        theKalmanFilter.predict(datum);
    }

    public void correct(KalmanDatum datum) {
        theKalmanFilter.correct(datum);
    }

    public Pose getPose() {
        return Pose.of(theKalmanFilter.outputVector());
    }

    public Pose getVelocity() {
        return theKalmanFilter.getRollingVelocity();
    }

    public Pose getOdometryVelocity() {
        return getOdometry().getRollingVelocity();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);
        data.add("Robot Pose: " + getPose().toDegrees());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();

        data.put("Sensor Thread Update time: ", "" + updateTime);
        data.put("Robot Pose Deg: ", getPose());

        return data;
    }

    @Override
    public String getName() {
        return "SensorThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    public Odometry getOdometry() {
        return odometry;
    }

    public IMU getImu() {
        return imu;
    }
}
