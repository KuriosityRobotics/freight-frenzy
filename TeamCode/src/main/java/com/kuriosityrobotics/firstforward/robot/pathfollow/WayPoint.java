package com.kuriosityrobotics.firstforward.robot.pathfollow;

import androidx.annotation.NonNull;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import java.util.ArrayList;

public class WayPoint extends Point {
    final AngleLock angleLock;
    final VelocityLock velocityLock;
    final ArrayList<Action> actions;

    public WayPoint(double x, double y, AngleLock angleLock, VelocityLock velocityLock, ArrayList<Action> actions) {
        super(x, y);

        this.angleLock = angleLock;
        this.velocityLock = velocityLock;

        this.actions = actions;
    }

    public WayPoint(double x, double y, double heading, double velo, ArrayList<Action> actions) {
        this(x, y, new AngleLock(heading), new VelocityLock(velo), actions);
    }

    public WayPoint(double x, double y, double heading, double velo) {
        this(x, y, heading, velo, new ArrayList<>());
    }

    public WayPoint(double x, double y, double velo, ArrayList<Action> actions) {
        this(x, y, new AngleLock(), new VelocityLock(velo), actions);
    }

    public WayPoint(Point point, double velo, ArrayList<Action> actions) {
        this(point.x, point.y, velo, actions);
    }

    public WayPoint(Pose pose, double velo, ArrayList<Action> actions){
        this(pose.x, pose.y, pose.heading, velo, actions);
    }

    public WayPoint(double x, double y, AngleLock angleLock) {
        this(x, y, angleLock, new VelocityLock(), new ArrayList<>());
    }

    public WayPoint(Pose pose){
        this(pose.x, pose.y, new AngleLock(pose.heading));
    }

    public WayPoint(double x, double y, VelocityLock velocityLock) {
        this(x, y, new AngleLock(), velocityLock, new ArrayList<>());
    }

    public WayPoint(double x, double y, ArrayList<Action> actions) {
        this(x, y, new AngleLock(), new VelocityLock(), actions);
    }

    public WayPoint(double x, double y) {
        this(x, y, new AngleLock(), new VelocityLock(), new ArrayList<>());
    }

    public WayPoint(Point point) {
        this(point.x, point.y);
    }

    public WayPoint(Point point, ArrayList<Action> actions) {
        this(point.x, point.y, actions);
    }

    public WayPoint(Point point, Action action) {
        this(point);
        this.actions.add(action);
    }

    public AngleLock getAngleLock() {
        return angleLock;
    }

    public VelocityLock getVelocityLock() {
        return velocityLock;
    }

    public ArrayList<Action> getActions() {
        return this.actions;
    }

    @NonNull
    @Override
    public String toString() {
        return "WayPoint{" +
                "x=" + x +
                ", y=" + y +
                ", angleLock=" + angleLock +
                ", velocityLock=" + velocityLock +
                ", actions=" + actions +
                '}';
    }
}
