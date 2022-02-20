package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.doublesEqual;

import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;

import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamMarkerDetector implements OpenCvConsumer {
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
    public static AutoStartLocation startLocation = AutoStartLocation.RED_CYCLE; // default

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.LEVEL_3 : location;
    }

    int runCount = 0;
    @Override
    public void processFrame(Mat frame) {
        // lol weird hack so we only run detect once but not the first time
        if (runCount != 20) {
            runCount++;
            return;
        }

        location = null;
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        // for 640x480
        final Rect boundingBox1;
        final Rect boundingBox2;
        final Rect boundingBox3;

        if (startLocation.equals(AutoStartLocation.RED_CYCLE) || startLocation.equals(AutoStartLocation.BLUE_DUCKS)) {
            boundingBox1 = new Rect(new Point(275, 220), new Point(380, 315));
            boundingBox2 = new Rect(new Point(475, 220), new Point(550, 315));
            boundingBox3 = new Rect(new Point(675, 220), new Point(790, 315));
        } else {
            boundingBox1 = new Rect(new Point(0, 220), new Point(125, 315));
            boundingBox2 = new Rect(new Point(255, 220), new Point(325, 315));
            boundingBox3 = new Rect(new Point(430, 220), new Point(525, 315));
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