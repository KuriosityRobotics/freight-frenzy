package com.kuriosityrobotics.firstforward.robot.sensors;

import static de.esoco.coroutine.Coroutine.*;
import static de.esoco.coroutine.step.CodeExecution.*;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.configuration.Configurator;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import de.esoco.coroutine.Coroutine;
import de.esoco.coroutine.CoroutineScope;

public class SensorThread implements Runnable {
    private static final Coroutine<SensorTick, Void> tickCoroutine = first(consume(SensorTick::tick));

    private final String configLocation;
    private final Robot robot;

    public static odo odo;
    private final SensorTick[] ticks;

    public SensorThread(Robot robot, String configLocation) {
        this.robot = robot;
        this.configLocation = configLocation;

        odo = new odo(robot);
        this.ticks = new SensorTick[]{odo};
    }


    @Override
    public void run() {
        Configurator.loadConfig(configLocation, ticks);

        while (!Thread.interrupted()) {
            CoroutineScope.launch( // Will block until all coroutines launched within this scope are done.
                    scope -> {
                        for (var tick : ticks) {
                            tickCoroutine.runAsync(scope, tick);
                        }
                    }
            );
        }
    }

    // reflection my beloved
    public HashMap<Class<?>, Map<String, String>> getSensorFields() {
        HashMap<Class<?>, Map<String, String>> map = new HashMap<>();
        for (var sensorTickable : ticks) {
            map.put(sensorTickable.getClass(),
                    Arrays.stream(sensorTickable.getClass().getFields())
                            .filter(field -> (field.getModifiers() & Modifier.VOLATILE) != 0) // assume volatile variables are buffered sensor values :lemonthink:
                            .collect(Collectors.toMap(Field::getName, n -> {
                                try {
                                    n.setAccessible(true);
                                    return n.get(sensorTickable).toString();
                                } catch (IllegalAccessException e) {
                                    e.printStackTrace();
                                }
                                return null;
                            })));
        }

        return map;
    }
}
