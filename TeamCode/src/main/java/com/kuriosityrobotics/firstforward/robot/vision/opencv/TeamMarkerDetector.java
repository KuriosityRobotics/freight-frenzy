package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.vision.PhysicalCamera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class TeamMarkerDetector implements OpenCvConsumer {
    // remember to set in auto!!!!!!!!!!!!!!!
    public static AutoStartLocation startLocation = AutoStartLocation.BLUE_CYCLE;
    int runCount = 0;
    private volatile TeamMarkerLocation location;

    public TeamMarkerDetector() {
    }

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.LEVEL_3 : location;
    }

    private boolean active = true;

    public void deactivate() {
        active = false;
    }

    /**
     * dont look too hard at this one
     * @param _img input frame
     */
    public void processFrame(Mat _img) {
        if (!active)
            return;

        var img = _img.clone();
        Core.bitwise_not(img, img);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2HSV);
        var bounding1 = new Rect(314, 156, 65, 86);
        var bounding2 = new Rect(454, 157, 71, 86);
        Imgproc.rectangle(img, bounding1, new Scalar(0, 0, 0));
        Imgproc.rectangle(img, bounding2, new Scalar(255, 255, 255));

        Core.inRange(img, new Scalar(90 - 10, 70, 50), new Scalar(90 + 10, 255, 255), img);

        var sub1 = img.submat(bounding1);
        var sub2 = img.submat(bounding2);

        var p1 = Core.countNonZero(sub1) / ((double) sub1.width() * sub1.height());
        var p2 = Core.countNonZero(sub2) / ((double) sub2.width() * sub2.height());
        var isSub1 = p1 > .4;
        var isSub2 = p2 > .4;

        sub1.release();
        sub2.release();
        img.release();

        if (isSub1)
            Imgproc.rectangle(_img, bounding1, new Scalar(0, 0, 0));

        if (isSub2)
            Imgproc.rectangle(_img, bounding2, new Scalar(0, 0, 0));


        if (isSub1 && isSub2) {
            if (p1 > p2)
                isSub2 = false;
            else
                isSub1 = false;
        }

        if (!isSub1 && !isSub2)
            this.location = Robot.isBlue ? TeamMarkerLocation.LEVEL_1 : TeamMarkerLocation.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3.LEVEL_3;
        else if (isSub1)
            this.location = Robot.isBlue ? TeamMarkerLocation.LEVEL_3 : TeamMarkerLocation.LEVEL_1;
        else
            this.location = TeamMarkerLocation.LEVEL_2;

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
                    return OuttakeModule.VerticalSlideLevel.DOWN_NO_EXTEND;
                case LEVEL_2:
                    return OuttakeModule.VerticalSlideLevel.MID;
                case LEVEL_3:
                    return OuttakeModule.VerticalSlideLevel.TOP;
            }
            return OuttakeModule.VerticalSlideLevel.MID;
        }
    }
}