package com.kuriosityrobotics.firstforward.robot.util.wrappers;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.mean;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.truncate;
import static java.util.concurrent.CompletableFuture.runAsync;

import android.annotation.SuppressLint;
import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.util.Timer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.ForkJoinPool;

public class AsynchProcess implements Telemeter {
    private final HashMap<Long, Double> updateTimes = new HashMap<>();
    private final ArrayList<AsynchProcess> chained = new ArrayList<>();
    private final Module module;
    private final Executor executor;

    private Timer timer;
    private CompletableFuture<Void> future;

    private double lastUpdate = 0;

    private AsynchProcess(Module module, Executor executor) {
        this.module = module;
        this.executor = executor;
    }

    public static AsynchProcess parallel(String name, Runnable task) {
        return new AsynchProcess(new Module() {
            @Override
            public void update() {
                task.run();
            }

            @Override
            public boolean isOn() {
                return true;
            }

            @Override
            public String getName() {
                return name;
            }
        }, ForkJoinPool.commonPool());
    }

    public static AsynchProcess parallel(String name, Runnable task, int maxFrequency) {
        return new AsynchProcess(new Module() {
            @Override
            public void update() {
                task.run();
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
            public int maxFrequency() {
                return maxFrequency;
            }
        }, ForkJoinPool.commonPool());
    }

    public static AsynchProcess parallel(Module module) {
        return new AsynchProcess(module, ForkJoinPool.commonPool());
    }

    public static AsynchProcess blocking(Module module) {
        return new AsynchProcess(module, Runnable::run);
    }

    public AsynchProcess chain(Module module) {
        chained.add(AsynchProcess.blocking(module));

        return this;
    }

    private void pushUpdateTime(double updateTime) {
        var now = SystemClock.elapsedRealtime();
        updateTimes.put(now, updateTime);
        updateTimes.entrySet().removeIf(entry -> entry.getKey() < now - 1000);

        lastUpdate = updateTime;
    }

    private long getMillisPerCycle() {
        if (module.maxFrequency() == 0)
            return 0;
        else
            return (long) (1000.) / module.maxFrequency();
    }

    public final synchronized void update() {
        if (future == null || (timer.tick() && future.isDone())) {
            if (timer != null)
                pushUpdateTime(timer.millisElapsedSinceStart());

            timer = Timer.alarmIn(getMillisPerCycle());
            future = runAsync(module::update, executor);
        }

        for (AsynchProcess asynchProcess : chained) {
            asynchProcess.update();
        }
    }

    public final synchronized double rollingAverageUpdateTime() {
        return mean(updateTimes.values());
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
//        double updateTime = rollingAverageUpdateTime();
        double updateTime = lastUpdate;

        return "Update time: " + truncate(updateTime, 1) + " ms (" + truncate(1000./updateTime, 1) + ").";
//        return String.format("update time:  %.1f ms (%.1f Hz)", updateTime, 1000. / updateTime);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        var lines = new ArrayList<String>(1 + chained.size());
        lines.add(toString());
        lines.addAll(module.getTelemetryData());

        for (AsynchProcess asynchProcess : chained) {
            var chainedTelemetry = asynchProcess.getTelemetryData();
            if (!chainedTelemetry.isEmpty())
                lines.add("-->" + asynchProcess.getName());

            chainedTelemetry.stream().map(line -> "   " + line)
                    .forEach(lines::add);
        }

        return lines;
    }

    @Override
    public String getName() {
        return module.getName();
    }

    @Override
    public boolean isOn() {
        return true;
    }
}
