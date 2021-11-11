package com.kuriosityrobotics.firstforward.robot.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.kuriosityrobotics.firstforward.robot.math.Pose;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */

// Imagine having to try to transfer KOTLIN code to java
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 6; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    // sadly not going to work :(
//    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
//        int samples = (int) Math.ceil(path.length() / resolution);
//        double[] xPoints = new double[samples];
//        double[] yPoints = new double[samples];
//        double dx = path.length() / (samples - 1);
//        for (int i = 0; i < samples; i++) {
//            double displacement = i * dx;
//            Pose2d pose = path.get(displacement);
//            xPoints[i] = pose.getX();
//            yPoints[i] = pose.getY();
//        }
//        canvas.strokePolyline(xPoints, yPoints);
//    }
//
//    public static void drawSampledPath(Canvas canvas, Path path) {
//        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
//    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.getHeadingVector().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}