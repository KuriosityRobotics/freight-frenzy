package com.kuriosityrobotics.firstforward.robot.util;

import org.apache.commons.math3.linear.RealMatrix;

public class SimpleMatrixFormatter {
    public static String toPoseString(RealMatrix m){
        StringBuilder sb = new StringBuilder();
        appendPoseValue(m, "x", "inches",0,0, sb);
        appendPoseValue(m, "y", "inches",1, 0, sb);
        appendPoseValue(m, "heading", "degrees",2, 0, sb);

        return sb.toString();
    }

    public static String toSTDString(RealMatrix m){
        StringBuilder sb = new StringBuilder();
        appendCovarianceValue(m, "dx", "inches",0,0, sb);
        appendCovarianceValue(m, "dy", "inches",1, 0, sb);
        appendCovarianceValue(m, "dheading", "degrees",2, 0, sb);

        return sb.toString();
    }

    private static void appendPoseValue(RealMatrix m, String name, String unit, int row, int column, StringBuilder sb) {
        sb.append("\n");
        sb.append(name).append(": ");
        if (m != null) {
            sb.append(m.getEntry(row, column));
        } else {
            sb.append("null");
        }
        sb.append(" ").append(unit).append("\n");
    }

    private static void appendCovarianceValue(RealMatrix m, String name, String unit, int row, int column, StringBuilder sb) {
        sb.append("\n");
        sb.append(name).append(": ");
        if (m != null) {
            sb.append(Math.sqrt(m.getEntry(row, column)));
        } else {
            sb.append("null");
        }
        sb.append(" ").append(unit).append("\n");
    }
}
