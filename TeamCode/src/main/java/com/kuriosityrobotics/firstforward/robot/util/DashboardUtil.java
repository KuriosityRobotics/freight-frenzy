package com.kuriosityrobotics.firstforward.robot.util;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Dashboard.ROBOT_RADIUS;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Webcam.*;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.kuriosityrobotics.firstforward.robot.math.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Set of helper functions for drawing Kuriosity robot and location history on dashboard canvases.
 */

public class DashboardUtil {
    public static void drawPoseHistory(Canvas canvas, List<Pose> poseHistory) {
        canvas.setStrokeWidth(1);
        canvas.setStroke("#3F51B5");
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        canvas.setStrokeWidth(1);
        canvas.setStroke("4CAF50");

        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.getHeadingVector().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}