package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import android.util.Log;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;

public class TeamMarkerDetection implements OpenCvConsumer {
    private static final Vector3D RED = new Vector3D(255, 0, 0);

    public enum TeamMarkerLocation {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3
    }

    private long lastCaptureTime = 0;

    private volatile TeamMarkerLocation location;

    public TeamMarkerLocation getLocation() {
        return location;
    }

    private static double rgbaToRgb(double value[])[] {
        return new double[]{ value[0], value[1], value[2] };
    }

    @Override
    public void processFrame(Mat frame) {
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);

        // width is 1920 height is 1080
        // these are tuned, both accurate AND consistent
        final Rect boundingBox1 = new Rect(new Point(284,482), new Point(475,765));
        final Rect boundingBox2 = new Rect(new Point(684,482), new Point(950,765));
        final Rect boundingBox3 = new Rect(new Point(1184, 482), new Point(1425,765));

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
            this.location = TeamMarkerLocation.LOCATION_1;
        } else if (closestToRed == area2) {
            Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 0), 5);
            this.location = TeamMarkerLocation.LOCATION_2;
        } else if (closestToRed == area3) {
            Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 0), 5);
            this.location = TeamMarkerLocation.LOCATION_3;
        }

        long millis = System.currentTimeMillis();
        if (millis - lastCaptureTime > 500) {
            File file = new File(AppUtil.ROBOT_DATA_DIR + "/" + "webcam-frame-" + new Date().getTime() + ".jpg");
            MatOfByte mob = new MatOfByte();
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2BGR);
            Imgcodecs.imencode(".jpg", frame, mob);
            try (FileOutputStream stream = new FileOutputStream(file)) {
                stream.write(mob.toArray());
            } catch (IOException e) {
                Log.w("VisionDump", e);
            }
            lastCaptureTime = millis;
        }
    }
}