package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Detect.*;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Max;
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
import java.util.ArrayList;
import java.util.Date;

public class TeamMarkerDetector implements OpenCvConsumer, Telemeter {
    private boolean isOn;

    public TeamMarkerDetector() {
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
        // yeah i'm sure these are correct(corresponding shipping hub levels)
        LEVEL_1,
        LEVEL_2,
        LEVEL_3;

        public OuttakeModule.VerticalSlideLevel slideLevel() {
            switch (this) {
                case LEVEL_1:
                    return OuttakeModule.VerticalSlideLevel.DOWN;
                case LEVEL_2:
                    return OuttakeModule.VerticalSlideLevel.MID;
                case LEVEL_3:
                    return OuttakeModule.VerticalSlideLevel.TOP;
            }
            return OuttakeModule.VerticalSlideLevel.MID;
        }
    }


    private volatile TeamMarkerLocation location;

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.LEVEL_3 : location;
    }

    private static double[] rgbaToRgb(double[] value) {
        return new double[] {value[0], value[1], value[2]};
    }

    int i = 0;
    @Override
    public void processFrame(Mat frame) {
        while (i < 20) {
            location = null;

            // TODO: Find out why PlebDetect streams at 1920x1080 but TeleOp streams at 640x480 with the exact same parameters???
            // width is 1920 height is 1080
            // these are tuned, both accurate AND consistent

            // for 1920x1080, DO NOT USE
//            final Rect boundingBox1 = new Rect(new Point(482, 284), new Point(765, 475));
//            final Rect boundingBox2 = new Rect(new Point(482, 684), new Point(765, 950));
//            final Rect boundingBox3 = new Rect(new Point(482, 1184), new Point(765, 1425));

            // for 640x480
            final Rect boundingBox1;
            final Rect boundingBox2;
            final Rect boundingBox3;

            if(!Robot.isBlue) {
                boundingBox1 = new Rect(new Point(0, 215), new Point(200, 415));
                boundingBox2 = new Rect(new Point(200, 215), new Point(400, 415));
                boundingBox3 = new Rect(new Point(400, 215), new Point(600, 415));
            } else {
                boundingBox2 = new Rect(new Point(640-200, 215), new Point(640-0, 415));
                boundingBox3 = new Rect(new Point(640-400, 215), new Point(640-200, 415));
                boundingBox1 = new Rect(new Point(640-600, 215), new Point(640-400, 415));
            }

            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

            Mat submat1 = frame.submat(boundingBox1);
            Mat submat2 = frame.submat(boundingBox2);
            Mat submat3 = frame.submat(boundingBox3);

            Vector3D sectionOne = new Vector3D(rgbaToRgb(Core.mean(submat1).val));
            Vector3D sectionTwo = new Vector3D(rgbaToRgb(Core.mean(submat2).val));
            Vector3D sectionThree = new Vector3D(rgbaToRgb(Core.mean(submat3).val));

            submat1.release();
            submat2.release();
            submat3.release();

            double area1 = Math.abs(angleWrap(Math.toRadians(sectionOne.getX())));
            double area2 = Math.abs(angleWrap(Math.toRadians(sectionTwo.getX())));
            double area3 = Math.abs(angleWrap(Math.toRadians(sectionThree.getX())));

            Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 0), 5);

            Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 0), 5);

            Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 0), 5);


            double closestToRed = new Max().evaluate(new double[]{area1, area2, area3});
            if (closestToRed == area1) {
                Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 255), 5);
                this.location = TeamMarkerLocation.LEVEL_1;
            } else if (closestToRed == area2) {
                Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 255), 5);
                this.location = TeamMarkerLocation.LEVEL_2;
            } else if (closestToRed == area3) {
                Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 255), 5);
                this.location = TeamMarkerLocation.LEVEL_3;
            }

            Log.v("Detect", "Detection finished. Location detected: " + location);

            i++;
        }
//        isOn = false;
    }
}