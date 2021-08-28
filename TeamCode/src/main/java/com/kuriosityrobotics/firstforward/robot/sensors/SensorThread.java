package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.*;
import static de.esoco.coroutine.CoroutineScope.launch;
import static de.esoco.coroutine.step.CodeExecution.*;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.configuration.Configurator;
import com.qualcomm.hardware.lynx.LynxModule;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import de.esoco.coroutine.Coroutine;

public class SensorThread implements Runnable {
    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final String configLocation;
    private final Robot robot;

    public static OdoSensors odoSensors;
    private final OdoProcessing odoProcessing;


    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;
        this.configLocation = configLocation;

        odoSensors = new OdoSensors(robot);
        odoProcessing = new OdoProcessing(robot);
    }


    @Override
    public void run() {
        while (!Thread.interrupted()) {
            launch(scope -> {
                bulkDataCoroutine.runAsync(scope, robot.revHub1);
                bulkDataCoroutine.runAsync(scope, robot.revHub2);
            });

            odoProcessing.process(odoSensors.get());
        }
    }

}
