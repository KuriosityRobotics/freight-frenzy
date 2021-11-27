package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;

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
        Iterator<Action> i = executing.iterator();
        while (i.hasNext()) {
            Action action = i.next();

            action.tick();

            if (action.isCompleted()) {
                i.remove();
            }
        }
    }

    public boolean doneExecuting() {
        return this.executing.isEmpty();
    }
}
