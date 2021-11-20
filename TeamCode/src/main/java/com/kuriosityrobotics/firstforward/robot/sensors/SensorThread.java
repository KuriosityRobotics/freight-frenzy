package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.util.MatrixUtil;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.qualcomm.hardware.lynx.LynxModule;

import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.HashMap;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable, Telemeter {
    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final String configLocation;
    private final Robot robot;

    public final Odometry odometry;
    private final LocalizeKalmanFilter kalmanFilter;

    private long updateTime = 0;
    private long lastLoopTime = 0;
    private long lastPoseSendTime = 0;

    private final LocalizationConsumer localizationConsumer;

    public SensorThread(Robot robot, String configLocation, LocalizationConsumer localizationConsumer) {
        this.robot = robot;
        this.configLocation = configLocation;
        this.localizationConsumer = localizationConsumer;

        robot.telemetryDump.registerTelemeter(this);

        this.odometry = new Odometry(robot);
        this.kalmanFilter = new LocalizeKalmanFilter(robot, MatrixUtil.ZERO_MATRIX);
    }


    @Override
    public void run() {
        while (robot.running()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
//                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });
            odometry.update();

            RealMatrix odometry = this.odometry.getDeltaMatrix();
            RealMatrix vuforia = this.localizationConsumer.getFormattedMatrix();

            this.kalmanFilter.update(odometry, vuforia);

            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - lastPoseSendTime >= 250) {
                robot.telemetryDump.sendPose(this.kalmanFilter.getFormattedPose());
                lastPoseSendTime = currentTime;
            }

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);
        data.add("Robot Pose: " + this.kalmanFilter.getFormattedPose());

        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("Sensor Thread Update time: ",  updateTime);
        data.put("Robot Pose: ",  this.kalmanFilter.getFormattedPose());

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
