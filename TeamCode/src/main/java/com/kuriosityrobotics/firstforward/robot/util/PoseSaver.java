package com.kuriosityrobotics.firstforward.robot.util;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.*;
import java.util.StringTokenizer;

public class PoseSaver {
    private static Pose savedPoseFromOpMode;
    private static SavedLocationMethod saveMethod;

    public enum SavedLocationMethod {
        FILE,
        OP_MODE,
        FUSE,
        NOT_SAVED
    }

    public static void savePose(Pose parkLoc) throws IOException {
        removePose();
        savedPoseFromOpMode = parkLoc;
        PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(AppUtil.ROBOT_DATA_DIR + "/" + "out.txt")));

        writer.println(parkLoc);
        writer.close();
    }

    public static void removePose() throws IOException {
        PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(AppUtil.ROBOT_DATA_DIR + "/" + "out.txt")));
        savedPoseFromOpMode = null;

        writer.println();
        writer.close();
    }

    public static Pose readPose() throws IOException {
        Pose parkLoc;

        BufferedReader reader = new BufferedReader(new FileReader(AppUtil.ROBOT_DATA_DIR + "/" + "out.txt"));

        StringTokenizer tokenizer = new StringTokenizer(reader.readLine(), " ,()");

        double x = Double.parseDouble(tokenizer.nextToken());
        double y = Double.parseDouble(tokenizer.nextToken());
        double headingRadians = Double.parseDouble(tokenizer.nextToken());

        parkLoc = new Pose(x, y, headingRadians);

        // redundancy 9999999999999999999999999999999999999999999999999
        // since we have both a pose from a saved file and a pose from the program

        Pose retPose;

        if (savedPoseFromOpMode != null && parkLoc != null) {
            retPose = Pose.fuse(savedPoseFromOpMode, parkLoc);
            saveMethod = SavedLocationMethod.FUSE;
        } else if (savedPoseFromOpMode != null && parkLoc == null) {
            retPose = savedPoseFromOpMode;
            saveMethod = SavedLocationMethod.OP_MODE;
        }
        else if (savedPoseFromOpMode == null && parkLoc != null) {
            retPose = parkLoc;
            saveMethod = SavedLocationMethod.FILE;
        }
        else {
            // TODO: should we have weird nudging cases like this?
            retPose = Pose.DEFAULT_POSE;
            saveMethod = SavedLocationMethod.NOT_SAVED;
        }

        return retPose;
    }

    public static SavedLocationMethod getSaveMethod() {
        return saveMethod;
    }
}
