package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;

import java.util.ArrayList;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleThread implements Runnable  {

    final boolean SHOW_UPDATE_SPEED = true;

    Robot robot;

    long lastUpdateTime = 0;

    public ModuleThread(Robot robot) {
//        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
    }

    /**
     * Gets all modules from robot, then runs update on them.
     */
    public void run() {
        while (robot.isOpModeActive()) {
            long currentIterationStartTime = SystemClock.elapsedRealtime();

            robot.update();

            if (SHOW_UPDATE_SPEED) {
                lastUpdateTime = SystemClock.elapsedRealtime() - currentIterationStartTime;
            }

//            if (Robot.WILL_FILE_DUMP) {
//                robot.fileDump.doTick();
//            }
        }
        System.out.println("Module executor thread exited due to opMode no longer being active.");

    }

//    @Override
//    public ArrayList<String> getTelemetryData() {
//        ArrayList<String> data = new ArrayList<>();
//        data.add("Module Executor thread loop time: " + lastUpdateTime);
//        return data;
//    }
}