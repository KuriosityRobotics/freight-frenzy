package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import android.util.Log;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;

public class TeamMarkerDetection implements OpenCvConsumer, Telemeter {
    private static final Vector3D RED = new Vector3D(255, 0, 0);
    private boolean isOn;

    public TeamMarkerDetection() {
        this.isOn = true;
    }

    @Override
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Location: " + location.toString());

        return data;
    }

    @Override
    public String getName() {
        return "TeamMarkerDetection";
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    public enum TeamMarkerLocation {
        // yeah i'm decently sure these are correct(corresponding shipping hub levels)
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }

    private volatile TeamMarkerLocation location;

    public TeamMarkerLocation getLocation() {
        return location;
    }

    private static double[] rgbaToRgb(double[] rgbaValue) {
        return new double[] {rgbaValue[0], rgbaValue[1], rgbaValue[2]};
    }

    @Override
    public void processFrame(Mat frame) {
        if (isOn) {
            for (int i = 0; i < 10; i++) {
                Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);

                // width is 1920 height is 1080
                // these are tuned, both accurate AND consistent
                final Rect boundingBox1 = new Rect(new Point(284, 482), new Point(475, 765));
                final Rect boundingBox2 = new Rect(new Point(684, 482), new Point(950, 765));
                final Rect boundingBox3 = new Rect(new Point(1184, 482), new Point(1425, 765));

                Mat submat1 = frame.submat(boundingBox1);
                Mat submat2 = frame.submat(boundingBox2);
                Mat submat3 = frame.submat(boundingBox3);
                Vector3D sectionOne = new Vector3D(rgbaToRgb(Core.mean(submat1).val));
                Vector3D sectionTwo = new Vector3D(rgbaToRgb(Core.mean(submat2).val));
                Vector3D sectionThree = new Vector3D(rgbaToRgb(Core.mean(submat3).val));

                submat1.release();
                submat2.release();
                submat3.release();

                double area1 = sectionOne.crossProduct(RED).getNorm();
                double area2 = sectionTwo.crossProduct(RED).getNorm();
                double area3 = sectionThree.crossProduct(RED).getNorm();

                double closestToRed = new Min().evaluate(new double[]{area1, area2, area3});
                if (closestToRed == area1) {
                    Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 0), 5);
                    this.location = TeamMarkerLocation.LEVEL_1;
                } else if (closestToRed == area2) {
                    Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 0), 5);
                    this.location = TeamMarkerLocation.LEVEL_2;
                } else if (closestToRed == area3) {
                    Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 0), 5);
                    this.location = TeamMarkerLocation.LEVEL_3;
                }
            }
        }

        isOn = false;
        Log.v("Team Marker Detection", "Detection finished. Location detected: " + location);
    }
}