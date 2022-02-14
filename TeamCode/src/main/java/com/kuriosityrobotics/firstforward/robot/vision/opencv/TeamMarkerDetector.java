package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.apache.commons.math3.stat.descriptive.rank.Min;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

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

    public enum AutoStartLocation {
        RED_DUCKS,
        RED_CYCLE,
        BLUE_DUCKS,
        BLUE_CYCLE
    }

    public enum TeamMarkerLocation {
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
    // remember to set in auto!!!!!!!!!!!!!!!
    public volatile AutoStartLocation startLocation;

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.LEVEL_3 : location;
    }

    private static double[] rgbaToRgb(double[] value) {
        return new double[] {value[0], value[1], value[2]};
    }

    int runCount = 0;
    @Override
    public void processFrame(Mat frame) {
        // lol weird hack so we only run detect once but not the first time
        if (runCount != 1) {
            runCount++;
            return;
        }

        location = null;
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        // for 640x480
        final Rect boundingBox1;
        final Rect boundingBox2;
        final Rect boundingBox3;

        if (startLocation.equals(AutoStartLocation.RED_DUCKS) || startLocation.equals(AutoStartLocation.BLUE_CYCLE)) {
            boundingBox1 = new Rect(new Point(285, 220), new Point(370, 315));
            boundingBox2 = new Rect(new Point(485, 220), new Point(540, 315));
            boundingBox3 = new Rect(new Point(685, 220), new Point(780, 315));
        } else {
            boundingBox1 = new Rect(new Point(0, 220), new Point(115, 315));
            boundingBox2 = new Rect(new Point(245, 220), new Point(315, 315));
            boundingBox3 = new Rect(new Point(420, 220), new Point(515, 315));
        }

        Mat submat1 = frame.submat(boundingBox1);
        Mat submat2 = frame.submat(boundingBox2);
        Mat submat3 = frame.submat(boundingBox3);

        double submat1pxls = Core.mean(submat1).val[0];
        double submat2pxls = Core.mean(submat2).val[0];
        double submat3pxls = Core.mean(submat3).val[0];

        submat1.release();
        submat2.release();
        submat3.release();

        Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 0), 5);
        Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 0), 5);
        Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 0), 5);

        double mostRed = new Max().evaluate(new double[]{submat1pxls, submat2pxls, submat3pxls});

        if (doublesEqual(submat1pxls, mostRed)) {
            Imgproc.rectangle(frame, boundingBox1, new Scalar(0, 0, 255), 5);
            location = TeamMarkerLocation.LEVEL_1;
        } else if (doublesEqual(submat2pxls, mostRed)) {
            Imgproc.rectangle(frame, boundingBox2, new Scalar(0, 0, 255), 5);
            location = TeamMarkerLocation.LEVEL_2;
        } else {
            Imgproc.rectangle(frame, boundingBox3, new Scalar(0, 0, 255), 5);
            location = TeamMarkerLocation.LEVEL_3;
        }

        runCount++;
    }
}