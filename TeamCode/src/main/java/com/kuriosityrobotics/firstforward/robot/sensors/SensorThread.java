package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.wrappers.SharpIRDistance;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.HashMap;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable, Telemeter {

    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final Robot robot;

    public final Odometry odometry;

    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    /**
     * This is a singleton.  Pose position and history is persisted through this.
     * Remember to register it as a telemeter each time a new Robot is created.
     */
    private static final ExtendedKalmanFilter theKalmanFilter = new ExtendedKalmanFilter(0, 0, 0);
    private final IMU imu;
    private final DistanceSensorLocaliser distanceSensorLocaliser;

    public void resetPose(Pose pose) {
        odometry.setPose(pose);

        robot.telemetryDump.removeTelemeter(theKalmanFilter);

        theKalmanFilter.reset(pose.x, pose.y, pose.heading);
        imu.resetPose(pose);

        robot.telemetryDump.registerTelemeter(theKalmanFilter);
    }

    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;

        robot.telemetryDump.registerTelemeter(this);
        robot.telemetryDump.registerTelemeter(theKalmanFilter);

        this.imu = new IMU(robot.hardwareMap, theKalmanFilter);
        this.odometry = new Odometry(robot, robot.hardwareMap, Pose.of(theKalmanFilter.outputVector()), theKalmanFilter, imu);
        var frontLeft = new SharpIRDistance(robot.hardwareMap, "frontLeft");
        var backLeft = new SharpIRDistance(robot.hardwareMap, "backLeft");
        var frontRight = new SharpIRDistance(robot.hardwareMap, "frontRight");
        var backRight = new SharpIRDistance(robot.hardwareMap, "backRight");
        this.distanceSensorLocaliser = new DistanceSensorLocaliser(
                robot,
                theKalmanFilter,
                frontLeft,
                backLeft,
                frontRight,
                backRight
        );

        robot.telemetryDump.registerTelemeter(distanceSensorLocaliser);

    }

    private long lastIMUUpdateTime = 0;

    @Override
    public void run() {
        while (robot.running()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });
            odometry.update();

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.telemetryDump.sendPose(Pose.of(theKalmanFilter.outputVector()).toDegrees());
                lastPoseSendTime = currentTime;
            }

            if (currentTime - lastIMUUpdateTime > 50) {
                imu.update();
                distanceSensorLocaliser.update();
                lastIMUUpdateTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    public Pose getPose() {
        return Pose.of(theKalmanFilter.outputVector());
    }

    public static ExtendedKalmanFilter getTheKalmanFilter() {
        return theKalmanFilter;
    }

    public Pose getVelocity() {
        return theKalmanFilter.getRollingVelocity();
    }

    public Pose getOdometryVelocity(){
        return odometry.getRollingVelocity();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);
        data.add("Robot Pose: " + getPose().toDegrees());

        data.add("");
        data.add("-- Odometry --");
        data.addAll(odometry.getTelemetryData());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();

        data.put("Sensor Thread Update time: ", "" + updateTime);
        data.put("Robot Pose Deg: ", getPose().toDegrees());

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
}
