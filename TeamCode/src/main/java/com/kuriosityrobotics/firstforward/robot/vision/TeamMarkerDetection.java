package com.kuriosityrobotics.firstforward.robot.vision;

import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class TeamMarkerDetection implements OpenCvConsumer {
    private static final Rect boundingBox1 = new Rect(1, 2, 3, 4);
    private static final Rect boundingBox2 = new Rect(1, 2, 3, 4);
    private static final Rect boundingBox3 = new Rect(1, 2, 3, 4);
    private static final Vector3D RED = new Vector3D(255, 0, 0);

    public static enum TeamMarkerLocation {
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
        var sectionOne = new Vector3D(Core.mean(frame.submat(boundingBox1)).val);
        var sectionTwo = new Vector3D(Core.mean(frame.submat(boundingBox2)).val);
        var sectionThree = new Vector3D(Core.mean(frame.submat(boundingBox3)).val);

        var area1 = sectionOne.crossProduct(RED).getNorm();
        var area2 = sectionTwo.crossProduct(RED).getNorm();
        var area3 = sectionThree.crossProduct(RED).getNorm();

        double closestToRed = new Min().evaluate(new double[]{area1, area2, area3});
        if (closestToRed == area1)
            this.location = TeamMarkerLocation.LOCATION_1;
        else if (closestToRed == area2)
            this.location = TeamMarkerLocation.LOCATION_2;
        else if (closestToRed == area3)
            this.location = TeamMarkerLocation.LOCATION_3;
        else
            System.err.println("something is terribly broken");
    }
}
