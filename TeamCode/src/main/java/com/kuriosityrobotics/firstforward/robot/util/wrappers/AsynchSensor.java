package com.kuriosityrobotics.firstforward.robot.util.wrappers;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.mean;
import static java.util.concurrent.CompletableFuture.runAsync;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.util.Timer;

import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.CompletableFuture;

public class AsynchSensor {
    private final long minimumMillisPerCycle; // 1 / hz
    private final Runnable sensorProcess;
    private final HashMap<Long, Double> updateTimes = new HashMap<>();

    private Timer timer;
    private CompletableFuture<Void> future;

    public AsynchSensor(int maxFrequency, Runnable sensorProcess) {
        this.minimumMillisPerCycle = (long) (1000. / maxFrequency);
        this.sensorProcess = sensorProcess;
    }

    public AsynchSensor(Runnable sensorProcess) {
        this.minimumMillisPerCycle = 0;
        this.sensorProcess = sensorProcess;
    }

    private void pushUpdateTime(double updateTime) {
        var now = SystemClock.elapsedRealtime();
        updateTimes.put(now, updateTime);
        updateTimes.entrySet().removeIf(entry -> entry.getKey() < now - 1000);
    }

    public final synchronized void update() {
        if (future == null || (timer.tick() && future.isDone())) {
            if (timer != null)
                pushUpdateTime(timer.millisElapsedSinceStart());

            timer = Timer.alarmIn(minimumMillisPerCycle);
            future = runAsync(sensorProcess);
        }
    }

    public final synchronized double rollingAverageUpdateTime() {
        return mean(updateTimes.values());
    }
}
