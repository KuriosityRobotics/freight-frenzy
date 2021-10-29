package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.consume;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.LocalizationConsumer;
import com.qualcomm.hardware.lynx.LynxModule;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable, Telemeter {
    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final String configLocation;
    private final Robot robot;

    private final Odometry odometry;
    private final LocalizeKalmanFilter kalmanFilter;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    private final LocalizationConsumer localizationConsumer;

    public SensorThread(Robot robot, String configLocation, LocalizationConsumer localizationConsumer) {
        this.robot = robot;
        this.configLocation = configLocation;
        this.localizationConsumer = localizationConsumer;

        robot.telemetryDump.registerTelemeter(this);

        this.odometry = new Odometry(robot, new Point(0,0),0.0);
        this.kalmanFilter = new LocalizeKalmanFilter(robot, odometry.getWorldMatrix());
    }


    @Override
    public void run() {
        while (robot.running()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
//                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });
            this.odometry.update();

            RealMatrix odo = this.odometry.getDeltaMatrix();

            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            RealMatrix delta = normalize(odo, updateTime);

            RealMatrix obs = this.localizationConsumer.getFormattedMatrix();

            this.kalmanFilter.update(delta, obs);
        }

        Log.v("SensorThread", "Exited due to opMode no longer being active.");
    }

    private RealMatrix normalize(RealMatrix odo, long updateInterval) {
        return MatrixUtils.createRealMatrix(new double[][]{
                {odo.getEntry(0,0) * (updateTime)},
                {odo.getEntry(1,0) * (updateTime)},
                {odo.getEntry(2,0)}
        });
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Update time: " + updateTime);

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
