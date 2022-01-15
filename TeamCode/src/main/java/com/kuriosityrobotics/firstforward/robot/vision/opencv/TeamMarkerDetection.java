package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.FileDump;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.OpenCvConsumer;
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
        return new double[]{
                value[0],
                value[1],
                value[2]
        };
    }

    @Override
    public void processFrame(Mat frame) {
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);

        final Rect boundingBox1 = new Rect(new Point((int) (frame.width() / 5), (int) (frame.height() / 3)), new Point(2 * (int) (frame.width() / 5), (int) (2 * frame.height() / 3)));
        final Rect boundingBox2 = new Rect(new Point(2 * (int) (frame.width() / 5), (int) (frame.height() / 3)), new Point(3 * (int) (frame.width() / 5), (int) (2 * frame.height() / 3)));
        final Rect boundingBox3 = new Rect(new Point(3 * (int) (frame.width() / 5), (int) (frame.height() / 3)), new Point(4 * (int) (frame.width() / 5), (int) (2 * frame.height() / 3)));

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