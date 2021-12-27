package com.kuriosityrobotics.firstforward.robot.pathfollow;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;

import java.util.ArrayList;
import java.util.Iterator;

public class ActionExecutor {
    private ArrayList<Action> executing = new ArrayList<>();

    private final Robot robot;

    public ActionExecutor(Robot robot) {
        this.robot = robot;
    }

    public void execute(Action action) {
        executing.add(action);
    }

    public void execute(WayPoint point) {
        for (Action action : point.getActions()) {
            this.execute(action);
        }
    }

    public void tick() {
        synchronized(this) {
            Iterator<Action> i = executing.iterator();
            while (i.hasNext()) {
                Action action = i.next();

                action.tick(this.robot);

                if (action.isCompleted()) {
                    Log.v("action", ""+action.getClass().getName()+" completed");
                    i.remove();
                }
            }
        }
    }

    public boolean doneExecuting() {
        synchronized(this) {
            return this.executing.isEmpty();
        }
    }
}