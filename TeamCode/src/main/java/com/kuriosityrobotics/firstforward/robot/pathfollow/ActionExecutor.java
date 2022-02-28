package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.util.Log;

import java.util.ArrayList;
import java.util.Iterator;

public class ActionExecutor {
    private static final ArrayList<Action> executing = new ArrayList<>();

    private ActionExecutor() {}

    public static void execute(Action action) {
        synchronized (executing) {
            executing.add(action);
        }
    }

    public static void execute(WayPoint point) {
        for (Action action : point.getActions()) {
            execute(action);
        }
    }

    public static void tick() {
        synchronized(executing) {
            Iterator<Action> i = executing.iterator();
            while (i.hasNext()) {
                Action action = i.next();

                action.tick();

                if (action.isCompleted()) {
                    i.remove();
                }
            }
        }
    }

    public static boolean doneExecuting() {
        synchronized(executing) {
            return executing.isEmpty();
        }
    }

    public static void reset() {
        executing.clear();
    }
}
