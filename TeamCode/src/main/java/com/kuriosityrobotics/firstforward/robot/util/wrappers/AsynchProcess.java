package com.kuriosityrobotics.firstforward.robot.util.wrappers;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.mean;
import static java.text.MessageFormat.format;
import static java.util.concurrent.CompletableFuture.runAsync;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.util.Timer;

import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.ForkJoinPool;

public class AsynchProcess {
    private final long minimumMillisPerCycle; // 1 / hz
    private final Runnable task;
    private final HashMap<Long, Double> updateTimes = new HashMap<>();
    private final Executor executor;

    private Timer timer;
    private CompletableFuture<Void> future;

    private AsynchProcess(Executor executor, long minimumMillisPerCycle, Runnable task) {
        this.executor = executor;
        this.minimumMillisPerCycle =  minimumMillisPerCycle;
        this.task = task;
    }

    public static AsynchProcess parallel(Runnable task) {
        return new AsynchProcess(ForkJoinPool.commonPool(), 0, task);
    }

    public static AsynchProcess parallel(Runnable task, int maxFrequency) {
        return new AsynchProcess(ForkJoinPool.commonPool(), (long) (1000. / maxFrequency), task);
    }

    public static AsynchProcess blocking(Runnable task) {
        return new AsynchProcess(Runnable::run, 0, task);
    }

    public static AsynchProcess blocking(Runnable task, int maxFrequency) {
        return new AsynchProcess(Runnable::run, (long) (1000. / maxFrequency), task);
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
            future = runAsync(task, executor);
        }
    }

    public final synchronized double rollingAverageUpdateTime() {
        return mean(updateTimes.values());
    }

    @Override
    public String toString() {
        double updateTime = rollingAverageUpdateTime();
        return format("update time:  {1} ms ({2} Hz)", updateTime, 1000. / updateTime);
    }
}
