package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.util.MatrixUtil;
import com.qualcomm.hardware.lynx.LynxModule;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.HashMap;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable, Telemeter {

    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final String configLocation;
    private final Robot robot;

    private final Odometry odometry;

    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    /**
     * This is a singleton.  Pose position and history is persisted through this.
     * Remember to register it as a telemeter each time a new Robot is created.
     */
    private static LocalizeKalmanFilter theKalmanFilter;

    static {
        theKalmanFilter = new LocalizeKalmanFilter(MatrixUtils.createRealMatrix(new double[][]{
                {0},
                {0},
                {0}
        }));
    }

    public void resetPose(Pose pose) {
        odometry.setPose(pose);

        robot.telemetryDump.removeTelemeter(theKalmanFilter);

        theKalmanFilter = new LocalizeKalmanFilter(MatrixUtils.createRealMatrix(new double[][]{
                {pose.x},
                {pose.y},
                {pose.heading}
        }));

        robot.telemetryDump.registerTelemeter(theKalmanFilter);
    }

    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;
        this.configLocation = configLocation;

        robot.telemetryDump.registerTelemeter(this);
        robot.telemetryDump.registerTelemeter(theKalmanFilter);

        this.odometry = new Odometry(robot, theKalmanFilter.getPose());
    }

    @Override
    public void run() {
        while (robot.running()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });
            odometry.update();

            RealMatrix odometry = this.odometry.getDeltaMatrix();
            RealMatrix vuforia = null;
            if (robot.isCamera) {
                vuforia = robot.visionThread.getVuforiaMatrix();
            }

            theKalmanFilter.update(odometry, vuforia);

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.telemetryDump.sendPose(theKalmanFilter.getPose().toDegrees());
                lastPoseSendTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    public Pose getPose() {
        return theKalmanFilter.getPose();
    }

    public Pose getVelocity() {
        return theKalmanFilter.getRollingVelocity();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);
        data.add("Robot Pose: " + theKalmanFilter.getPose().toDegrees());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();

        data.put("Sensor Thread Update time: ", "" + updateTime);
        data.put("Robot Pose Deg: ", theKalmanFilter.getPose().toDegrees());

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
