package com.kuriosityrobotics.firstforward.robot.vision;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class TeamMarkerDetection implements OpenCvConsumer {
    static final Point TOP = new Point(0,0);
    static final Point MID_TOP = new Point(640, 0);
    static final Point END_TOP = new Point(1280,0);
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(640,1080);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(1280,1080);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1920,1080);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;
    private static final Vector3D RED = new Vector3D(255, 0, 0);

    public enum TeamMarkerLocation {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3
    }

    private volatile TeamMarkerLocation location;

    public TeamMarkerLocation getLocation() {
        return location;
    }

    @Override
    public void processFrame(Mat frame) {
        Mat region1 = frame.submat(new Rect(TOP, REGION1_TOPLEFT_ANCHOR_POINT));
        Mat region2 = frame.submat(new Rect(MID_TOP, REGION2_TOPLEFT_ANCHOR_POINT));
        Mat region3 = frame.submat(new Rect(END_TOP, REGION3_TOPLEFT_ANCHOR_POINT));

        var sectionOne = (new Vector3D(Core.mean(region1).val)).crossProduct(RED).getNorm();
        var sectionTwo = (new Vector3D(Core.mean(region2).val)).crossProduct(RED).getNorm();
        var sectionThree = (new Vector3D(Core.mean(region3).val)).crossProduct(RED).getNorm();

        double temp = Math.max(sectionOne, sectionTwo);
        double max = Math.max(temp, sectionThree);
        if (Math.abs(sectionOne - max) < 0.0001) {
            location = TeamMarkerLocation.LOCATION_1;
        } else if (Math.abs(sectionTwo - max) < 0.0001) {
            location = TeamMarkerLocation.LOCATION_2;
        } else if (Math.abs(sectionThree - max) < 0.0001){
            location = TeamMarkerLocation.LOCATION_3;
        } else {
            Log.e("Team marker detection", "something is terribly broken and should b fixed asap");
        }
    }
}