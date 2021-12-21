package com.kuriosityrobotics.firstforward.robot.util;

import com.kuriosityrobotics.firstforward.robot.math.Pose;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.*;
import java.util.StringTokenizer;

public class PoseSaver {
    public static void savePose(Pose parkLoc) throws IOException {
        PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(AppUtil.ROBOT_DATA_DIR + "/" + "out.txt")));

        writer.println(parkLoc);
        writer.close();
    }

    public static void removePose() throws IOException {
        PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(AppUtil.ROBOT_DATA_DIR + "/" + "out.txt")));

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
        removePose();

        return parkLoc;
    }
}
