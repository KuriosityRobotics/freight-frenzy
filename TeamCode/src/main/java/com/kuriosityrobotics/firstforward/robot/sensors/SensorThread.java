package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;
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
    private final LocalizeKalmanFilter kalmanFilter;

    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    private final VuforiaLocalizationConsumer vuforiaLocalizationConsumer;

    public SensorThread(Robot robot, String configLocation, VuforiaLocalizationConsumer vuforiaLocalizationConsumer, Pose pose) {
        this.robot = robot;
        this.configLocation = configLocation;
        this.vuforiaLocalizationConsumer = vuforiaLocalizationConsumer;

        robot.telemetryDump.registerTelemeter(this);

        this.odometry = new Odometry(robot, pose);
        this.kalmanFilter = new LocalizeKalmanFilter(robot, MatrixUtils.createRealMatrix(new double[][]{
                {pose.x},
                {pose.y},
                {pose.heading}
        }));
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
            RealMatrix vuforia = this.vuforiaLocalizationConsumer.getFormattedMatrix();

            this.kalmanFilter.update(odometry, vuforia);

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.telemetryDump.sendPose(this.kalmanFilter.getPoseRadians());
                lastPoseSendTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    public Pose getPose() {
        return kalmanFilter.getPoseRadians();
    }

    public Pose getVelocity() {
        return kalmanFilter.getRollingVelocity();
    }

    public Pose getOdomVelocity() {
        return odometry.getInstantaneousVelocity();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);
        data.add("Robot Pose Rad: " + this.kalmanFilter.getPoseRadians());
        data.add("Robot PoseDeg : " + this.kalmanFilter.getPoseDegrees());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("Sensor Thread Update time: ",  "" + updateTime);
        data.put("Robot Pose Rad: ",  this.kalmanFilter.getPoseRadians());
        data.put("Robot Pose Deg: ",  this.kalmanFilter.getPoseDegrees());

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
