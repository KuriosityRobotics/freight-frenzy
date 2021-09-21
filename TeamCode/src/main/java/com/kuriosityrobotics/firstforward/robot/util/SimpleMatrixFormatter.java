package com.kuriosityrobotics.firstforward.robot.util;

import org.apache.commons.math3.linear.RealMatrix;

public class SimpleMatrixFormatter {
    public static String toPoseString(RealMatrix m){
        StringBuilder sb = new StringBuilder();
        appendMatrixValue(m, "x", "inches",0,0, sb);
        appendMatrixValue(m, "y", "inches",1, 0, sb);
        appendMatrixValue(m, "heading", "inches",2, 0, sb);

        return sb.toString();
    }

    public static String toSTDString(RealMatrix m){
        // its the same
        return toPoseString(m);
    }

    private static void appendMatrixValue(RealMatrix m, String name, String unit, int row, int column, StringBuilder sb) {
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
