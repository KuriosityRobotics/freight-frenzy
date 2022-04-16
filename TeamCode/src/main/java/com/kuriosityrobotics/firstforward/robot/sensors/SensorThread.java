package com.kuriosityrobotics.firstforward.robot.sensors;

import static java.util.concurrent.ForkJoinPool.commonPool;
import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.KalmanDatum;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.AsynchProcess;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.SharpIRDistance;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

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

    private final HashSet<AsynchProcess> sensors;

    private final Robot robot;
    private final Odometry odometry;
    private final IMU imu;
    private final DistanceSensorLocaliser distanceSensorLocaliser;

    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    public SensorThread(Robot robot) {
        this.robot = robot;
        robot.getTelemetryDump().registerTelemeter(theKalmanFilter);

        this.imu = new IMU(robot.getHardwareMap(), theKalmanFilter);
        this.odometry = new Odometry(robot.getHardwareMap(), getPose(), theKalmanFilter, imu);

        var frontLeft = new SharpIRDistance(robot.getHardwareMap(), "frontLeft");
        var backLeft = new SharpIRDistance(robot.getHardwareMap(), "backLeft");
        var frontRight = new SharpIRDistance(robot.getHardwareMap(), "frontRight");
        var backRight = new SharpIRDistance(robot.getHardwareMap(), "backRight");
        this.distanceSensorLocaliser = new DistanceSensorLocaliser(
                robot,
                theKalmanFilter,
                frontLeft,
                backLeft,
                frontRight,
                backRight
        );

        sensors = new HashSet<>();
        sensors.add(AsynchProcess.parallel(imu));
        sensors.add(AsynchProcess.parallel("EH", robot.getExpansionHub()::getBulkData, 60));
        sensors.add(AsynchProcess.parallel("CH", robot.getControlHub()::getBulkData)
                .chain(odometry)
                .chain(frontLeft)
                .chain(backLeft)
                .chain(frontRight)
                .chain(backRight)
                .chain(distanceSensorLocaliser));

        sensors.forEach(robot.getTelemetryDump()::registerTelemeter);
    }

    public ExtendedKalmanFilter getKalmanFilter() {
        return theKalmanFilter;
    }

    public void resetPose(Pose pose) {
        getOdometry().setPose(pose);

        robot.getTelemetryDump().removeTelemeter(theKalmanFilter);

        theKalmanFilter.reset(pose.x, pose.y, pose.heading);
        getImu().resetPose(pose);

        robot.getTelemetryDump().registerTelemeter(theKalmanFilter);
    }

    @Override
    public void run() {
        while (robot.running()) {
            sensors.forEach(AsynchProcess::update);

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.getTelemetryDump().sendPose(getPose());
                lastPoseSendTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        commonPool().shutdown();
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
        data.add(getPose().toString("Robot pose"));
        data.add(Pose.of(theKalmanFilter.getVariance()).toString("variance"));

        return data;
    }

    @Override
    public int getShowIndex() {
        return 5;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();

        data.put("Sensor Thread Update time: ", "" + updateTime);
        data.put("Robot Pose: ", getPose());
        data.put("variance:  ", Pose.of(theKalmanFilter.getVariance()));

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
