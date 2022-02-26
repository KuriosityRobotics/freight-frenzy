package com.kuriosityrobotics.firstforward.robot.util;

import android.annotation.SuppressLint;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.text.DecimalFormat;

public class MatrixUtil {
    @SuppressLint("DefaultLocale")
    public static String toPoseString(RealMatrix m){
        return String.format("x:  %.2f, y:  %.2f, θ:  %.2f", m.getEntry(0, 0), m.getEntry(1, 0), m.getEntry(2, 0));
    }

    public static String toCovarianceString(RealMatrix m){
        if (m == null) {
            return "";
        }

        return "x" + m.getEntry(0, 0) +
                'y' + m.getEntry(1, 1) +
                't' + m.getEntry(2, 2);
    }
}
