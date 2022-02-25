package com.kuriosityrobotics.firstforward.robot.util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.text.DecimalFormat;

public class MatrixUtil {
    private static final DecimalFormat df = new DecimalFormat("#.000");

    public static String toPoseString(RealMatrix m){
        StringBuilder sb = new StringBuilder();
        appendPoseValue(m, "x", 0, sb);
        appendPoseValue(m, "y", 1, sb);
        if (m != null){
            sb.append("heading").append(": ").append(Math.toDegrees(m.getEntry(2, 0))).append(" ").append("degrees").append("\n");
        }
        else {
            sb.append("heading").append(": ").append("null").append(" ").append("degrees").append("\n");
        }

        return sb.toString();
    }

    public static String toCovarianceString(RealMatrix m){
        if (m == null) {
            return "";
        }

        return "x" + m.getEntry(0, 0) +
                'y' + m.getEntry(1, 1) +
                't' + m.getEntry(2, 2);
    }

    public static String toSTDString(RealMatrix m){
        StringBuilder sb = new StringBuilder();
        appendCovarianceValue(m, "dx", 0, sb);
        appendCovarianceValue(m, "dy", 1, sb);
        if (m != null){
            sb.append("dHeading").append(": ").append(Math.toDegrees(Math.sqrt(m.getEntry(2, 0)))).append(" ").append("degrees").append("\n");
        }
        else {
            sb.append("dHeading").append(": ").append("null").append(" ").append("degrees").append("\n");
        }

        return sb.toString();
    }

    private static void appendPoseValue(RealMatrix m, String name, int row, StringBuilder sb) {
        sb.append(name).append(": ");
        if (m != null) {
            sb.append(df.format(m.getEntry(row, 0)));
        } else {
            sb.append("null");
        }
        sb.append(" ").append("inches").append("\n");
    }

    private static void appendCovarianceValue(RealMatrix m, String name, int row, StringBuilder sb) {
        sb.append(name).append(": ");
        if (m != null) {
            sb.append(df.format(Math.sqrt(m.getEntry(row, 0))));
        } else {
            sb.append("null");
        }
        sb.append(" ").append("inches").append("\n");
    }

    public static RealMatrix ZERO_MATRIX = MatrixUtils.createRealMatrix(new double[][]{
            {0},
            {0},
            {0}
    });
}
