package com.kuriosityrobotics.firstforward.robot.util.wrappers;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.median;
import static java.lang.Math.exp;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.collections4.keyvalue.UnmodifiableMapEntry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SharpIRDistance implements Module {
    private static final long WINDOW_SIZE_MS = 70;
    private final AnalogInput analogInput;
    private final String name;
    private final ConcurrentLinkedDeque<Map.Entry<Long, Double>> measurements = new ConcurrentLinkedDeque<>();

    public SharpIRDistance(HardwareMap hardwareMap, String name) {
        this.analogInput = hardwareMap.get(AnalogInput.class, name);
        this.name = name;
    }

    private double voltageToIn(double voltage) {
        return 12.3615 * exp(-2.59267 * voltage) + 0.817458;
    }

    public void update() {
        var currentTime = SystemClock.elapsedRealtime();
        while (!measurements.isEmpty() && currentTime - measurements.peek().getKey() > WINDOW_SIZE_MS)
            measurements.poll();

        measurements.add(new UnmodifiableMapEntry<>(SystemClock.elapsedRealtime(), analogInput.getVoltage()));
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add(String.format("Distance:  %04.2f in", getDistance()));
            var latestMeasurement = measurements.peekLast();
            if (latestMeasurement != null)
                add(String.format("Voltage:  %04.2f V", latestMeasurement.getValue()));
        }};
    }

    public double getDistance() {
        return voltageToIn(median(measurements));
    }
}
