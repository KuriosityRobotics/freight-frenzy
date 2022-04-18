package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamMarkerDetector implements OpenCvConsumer {
    private volatile TeamMarkerLocation location;

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.UNKNOWN : location;
    }

    private boolean active = false;

    public void activate() {
        this.active = true;
    }
    public void deactivate() {
        active = false;
        location = null;
    }

    /**
     * dont look too hard at this one
     *
     * @param _img input frame
     */
    public void processFrame(double cameraAngle, Mat _img) {
        var bmp = Bitmap.createBitmap(_img.cols(), _img.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(_img, bmp);
        FtcDashboard.getInstance().sendImage(bmp);


        if (!active)
            return;

        var img = _img.clone();
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        Core.bitwise_not(img, img);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);

        var bounding1 = new Rect(333, 107, 86, 114);
        var bounding2 = new Rect(455, 107, 86, 114);
        Imgproc.rectangle(img, bounding1, new Scalar(0, 0, 0));
        Imgproc.rectangle(img, bounding2, new Scalar(255, 255, 255));

        Core.inRange(img, new Scalar(90 - 10, 70, 50), new Scalar(90 + 10, 255, 255), img);

        boolean badBound1 = !new Rect(0, 0, img.cols(), img.rows()).contains(bounding1.tl()) ||
                !new Rect(0, 0, img.cols(), img.rows()).contains(bounding1.br());
        boolean badBound2 = !new Rect(0, 0, img.cols(), img.rows()).contains(bounding2.tl()) ||
                !new Rect(0, 0, img.cols(), img.rows()).contains(bounding2.br());
        if (badBound1 || badBound2) {
            img.release();
            return;
        }

        var sub1 = img.submat(bounding1);
        var sub2 = img.submat(bounding2);

        var p1 = Core.countNonZero(sub1) / ((double) sub1.width() * sub1.height());
        var p2 = Core.countNonZero(sub2) / ((double) sub2.width() * sub2.height());
        var isSub1 = p1 > .2;
        var isSub2 = p2 > .2;

        sub1.release();
        sub2.release();
        img.release();

//        if (isSub1)


        if (isSub1 && isSub2) {
            if (p1 > p2)
                isSub2 = false;
            else
                isSub1 = false;
        }

        if (!isSub1 && !isSub2)
            this.location = Robot.isBlue ? TeamMarkerLocation.LEVEL_1 : TeamMarkerLocation.LEVEL_3;
        else if (isSub1)
            this.location = Robot.isBlue ? TeamMarkerLocation.LEVEL_3 : TeamMarkerLocation.LEVEL_1;
        else
            this.location = TeamMarkerLocation.LEVEL_2;
    }

    public enum TeamMarkerLocation {
        UNKNOWN,
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
                case UNKNOWN:
                    return OuttakeModule.VerticalSlideLevel.TOP;
            }
            return OuttakeModule.VerticalSlideLevel.MID;
        }
    }
}