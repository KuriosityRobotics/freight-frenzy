package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.*;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.*;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.hardware.lynx.LynxModule;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable {
    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final String configLocation;
    private final Robot robot;

    private final Odometry odometry;


    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;
        this.configLocation = configLocation;

        odometry = new Odometry(robot);
    }


    @Override
    public void run() {
        while (!Thread.interrupted()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });

            odometry.process();
        }
    }

}
