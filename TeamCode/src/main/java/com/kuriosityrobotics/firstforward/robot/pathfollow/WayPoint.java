package com.kuriosityrobotics.firstforward.robot.pathfollow;

import com.kuriosityrobotics.firstforward.robot.math.Point;

import java.util.ArrayList;

public class WayPoint extends Point {
    boolean lockHeading;
    double heading;

    boolean targetVelocity;
    double velocity;

    ArrayList<Action> actions;

    public WayPoint(double x, double y, boolean lockHeading, double heading, boolean targetVelocity, double velocity, ArrayList<Action> actions) {
        super(x, y);

        this.lockHeading = lockHeading;
        this.heading = heading;

        this.targetVelocity = targetVelocity;
        this.velocity = velocity;

        this.actions = actions;
    }

    public WayPoint(double x, double y, double heading, double velocity, ArrayList<Action> actions) {
        this(x, y, true, heading, true, velocity, actions);
    }

    public WayPoint(double x, double y, double heading, double velocity) {
        this(x, y, true, heading, true, velocity, new ArrayList<>());
    }

    public WayPoint(double x, double y, ArrayList<Action> actions) {
        this(x, y, false, 0, false, 0, actions);
    }

    public WayPoint(double x, double y, Action action) {
        this(x, y);

        actions.add(action);
    }

    public WayPoint(double x, double y) {
        this(x, y, 0, 0);
    }
}
