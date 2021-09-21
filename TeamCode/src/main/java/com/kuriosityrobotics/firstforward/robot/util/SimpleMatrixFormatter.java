package com.kuriosityrobotics.firstforward.robot.util;

import org.apache.commons.math3.linear.RealMatrix;

public class SimpleMatrixFormatter {
    public static String toPoseString(RealMatrix m){
        String retVal = "\n";
        retVal += "x: " + m.getEntry(0,0) + " inches\n";
        retVal += "y: " + m.getEntry(1,0) + " inches\n";
        retVal += "heading: " + Math.toDegrees(m.getEntry(2,0)) + " degrees\n";
        return retVal;
    }

    public static String toSTDString(RealMatrix m){
        String retVal = "\n";
        retVal += "x: " + Math.sqrt(m.getEntry(0,0)) + " inches\n";
        retVal += "y: " + Math.sqrt(m.getEntry(1,1)) + " inches\n";
        retVal += "heading: " + Math.toDegrees(Math.sqrt(m.getEntry(2,2))) + " degrees\n";
        return retVal;
    }
}
