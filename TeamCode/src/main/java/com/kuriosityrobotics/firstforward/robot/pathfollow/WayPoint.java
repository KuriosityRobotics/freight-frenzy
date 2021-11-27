package com.kuriosityrobotics.firstforward.robot.pathfollow;

import com.kuriosityrobotics.firstforward.robot.math.Point;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.Action;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class WayPoint extends Point {
    AngleLock angleLock;

    boolean targetVelocity;
    double velocity;

    final ArrayList<Action> actions;

    public WayPoint(double x, double y, AngleLock angleLock, boolean targetVelocity, double velocity, ArrayList<Action> actions) {
        super(x, y);

        this.angleLock = angleLock;

        this.targetVelocity = targetVelocity;
        this.velocity = velocity;

        this.actions = actions;
    }

    public WayPoint(double x, double y, double heading, double velocity, ArrayList<Action> actions) {
        this(x, y, new AngleLock(heading), true, velocity, actions);
    }

    public WayPoint(double x, double y, AngleLock angleLock, boolean targetVelocity, double velocity) {
        this(x, y, angleLock, targetVelocity, velocity, new ArrayList<>());
    }

    public WayPoint(double x, double y, AngleLock angleLock) {
        this(x, y, angleLock, false, 0);
    }

    public WayPoint(double x, double y, double heading, double velocity) {
        this(x, y, new AngleLock(heading), true, velocity);
    }

    public WayPoint(double x, double y, ArrayList<Action> actions) {
        this(x, y, new AngleLock(), false, 0, actions);
    }

    public WayPoint(double x, double y, double velocity) {
        this(x, y, new AngleLock(), true, velocity);
    }

    public WayPoint(double x, double y, AngleLock angleLock, Action action) {
        this(x, y, angleLock, false, 0, new ArrayList<>());
        this.actions.add(action);
    }

    public WayPoint(double x, double y, double heading, Action action) {
        this(x, y, new AngleLock(heading), action);
    }

    public WayPoint(double x, double y) {
        this(x, y, new ArrayList<>());
    }

    public WayPoint(double x, double y, Action action) {
        this(x, y);

        actions.add(action);
    }

    public AngleLock getAngleLock() {
        return angleLock;
    }

    public boolean hasTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return velocity;
    }

    public ArrayList<Action> getActions() {
        return this.actions;
    }
}
