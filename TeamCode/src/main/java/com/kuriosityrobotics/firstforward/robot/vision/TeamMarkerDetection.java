package com.kuriosityrobotics.firstforward.robot.vision;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class TeamMarkerDetection implements OpenCvConsumer {
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

    // TODO: write 4-D to 3-D since core.mean returns 4 dimensions but vector 3d only takes 3
    private static double rgbaToRgb(double value[])[] {
        return new double[]{
                value[0],
                value[1],
                value[2]
        };
    }

    @Override
    public void processFrame(Mat frame) {
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);

        final Rect boundingBox1 = new Rect(new Point(0, 0), new Point((int) (frame.width() / 3), frame.height()));
        final Rect boundingBox2 = new Rect(new Point((int) (frame.width() / 3), 0), new Point((int) (2 * frame.width() / 3), frame.height()));
        final Rect boundingBox3 = new Rect(new Point((int) (2 * frame.width() / 3), 0), new Point(frame.width(), frame.height()));

        var submat1 = frame.submat(boundingBox1);
        var submat2 = frame.submat(boundingBox2);
        var submat3 = frame.submat(boundingBox3);
        var sectionOne = new Vector3D(rgbaToRgb(Core.mean(submat1).val));
        var sectionTwo = new Vector3D(rgbaToRgb(Core.mean(submat2).val));
        var sectionThree = new Vector3D(rgbaToRgb(Core.mean(submat3).val));
        submat1.release();
        submat2.release();
        submat3.release();

        var area1 = sectionOne.crossProduct(RED).getNorm();
        var area2 = sectionTwo.crossProduct(RED).getNorm();
        var area3 = sectionThree.crossProduct(RED).getNorm();
        Log.d(getClass().getName(), "pepega");
        double closestToRed = new Min().evaluate(new double[]{area1, area2, area3});
        if (closestToRed == area1) {
            Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 0), 5);
            this.location = TeamMarkerLocation.LOCATION_1;
        } else if (closestToRed == area2) {
            Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 0), 5);
            this.location = TeamMarkerLocation.LOCATION_2;
        } else if (closestToRed == area3) {
            Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 0), 5);
            this.location = TeamMarkerLocation.LOCATION_3;
        } else
            System.err.println("something is terribly broken");

    }
}