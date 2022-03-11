package com.kuriosityrobotics.firstforward.robot.vision.opencv;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.kuriosityrobotics.firstforward.robot.LocationProvider;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.vision.minerals.PinholeCamera;

import org.apache.commons.geometry.euclidean.threed.PlaneConvexSubset;
import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.shape.Parallelepiped;
import org.apache.commons.geometry.euclidean.twod.Bounds2D;
import org.apache.commons.numbers.core.Precision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.stream.Collectors;

public class TeamMarkerDetector implements OpenCvConsumer {
    // remember to set in auto!!!!!!!!!!!!!!!
    private final LocationProvider locationProvider;
    int runCount = 0;
    private volatile TeamMarkerLocation location;

    private final PinholeCamera pinholeCamera;

    public TeamMarkerDetector(LocationProvider locationProvider) {
        this.locationProvider = locationProvider;
        this.pinholeCamera = PinholeCamera.create();
    }

    public TeamMarkerLocation getLocation() {
        return location == null ? TeamMarkerLocation.UNKNOWN : location;
    }

    private boolean active = true;

    public void deactivate() {
        active = false;
    }

/*    private Parallelepiped levelOne() {
        if (Robot.isBlue) {
            return Parallelepiped.builder(Precision.doubleEquivalenceOfEpsilon(.05))
                    .setPosition(Vector3D.of(35.25, 2.5, 144 - 73 - 1 + 8.38 + 8.38))
                    .setScale(4, 5, 4)
                    .build();
        } else {
            return Parallelepiped.builder(Precision.doubleEquivalenceOfEpsilon(.05))
                    .setPosition(Vector3D.of(35.25, 2.5, 144 - 73 - 1))
                    .setScale(4, 5, 4)
                    .build();
        }
    }*/

    private Vector3D[] levelOne(double cameraAngle) {
        if (Robot.isBlue) {
            return new Vector3D[]{
                    Vector3D.of(144 - 36 + 3, 0, 144 - 73.5 + 2.5 - 2.5).add(Vector3D.of(3.365f * sin(cameraAngle), 0, 3.365f * cos(cameraAngle))),
                    Vector3D.of(144 - 36 + 3, 6.5, 144 - 73.5 - 2.5 - 2.5).add(Vector3D.of(3.365f * sin(cameraAngle), 0, 3.365f * cos(cameraAngle)))
            };
        } else {
            return new Vector3D[]{
                    Vector3D.of(36 - 3, 0, 144 - 73.5 + 2.5 - 2.5).add(Vector3D.of(3.365f * sin(cameraAngle), 0, 3.365f * cos(cameraAngle))),
                    Vector3D.of(36 - 3, 6.5, 144 - 73.5 - 2.5 - 2.5).add(Vector3D.of(3.365f * sin(cameraAngle), 0, 3.365f * cos(cameraAngle)))
            };
        }
    }

    private Vector3D[] levelTwo(double cameraAngle) {
        var levelOne = levelOne(cameraAngle);
        if (Robot.isBlue) {
            return new Vector3D[]{
                    levelOne[0].add(Vector3D.of(0, 0, -8.35)),
                    levelOne[1].add(Vector3D.of(0, 0, -8.35))};
        } else {
            return new Vector3D[]{
                    levelOne[0].add(Vector3D.of(0, 0, -8.35)),
                    levelOne[1].add(Vector3D.of(0, 0, -8.35))
            };
        }
    }

    /**
     * dont look too hard at this one
     * @param _img input frame
     */
    public void processFrame(double cameraAngle, Mat _img) {
        if (!active)
            return;

        var img = _img.clone();
        Core.bitwise_not(img, img);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2HSV);

        var frameCamera = pinholeCamera.bindToPose(Vector3D.of(locationProvider.getPose().x, 0, locationProvider.getPose().y), cameraAngle);

        var pts1 = Arrays.stream(levelOne(angleWrap(cameraAngle /*- locationProvider.getPose().heading*/))).map(frameCamera::getLocationOnFrame)
                .map(n -> new Point(n.getX(), n.getY())).collect(Collectors.toList());

        var pts2 = Arrays.stream(levelTwo(angleWrap(cameraAngle /*- locationProvider.getPose().heading*/))).map(frameCamera::getLocationOnFrame)
                .map(n -> new Point(n.getX(), n.getY())).collect(Collectors.toList());

        var bounding1 = new Rect(pts1.get(0), pts1.get(1));
        var bounding2 = new Rect(pts2.get(0), pts2.get(1));
        Imgproc.rectangle(img, bounding1, new Scalar(0, 0, 0));
        Imgproc.rectangle(img, bounding2, new Scalar(255, 255, 255));

        Core.inRange(img, new Scalar(90 - 10, 70, 50), new Scalar(90 + 10, 255, 255), img);

        if (!new Rect(0, 0, img.cols(), img.rows()).contains(bounding1.tl()) ||
                !new Rect(0, 0, img.cols(), img.rows()).contains(bounding1.br()))
            return;

        var sub1 = img.submat(bounding1);
        var sub2 = img.submat(bounding2);

        var p1 = Core.countNonZero(sub1) / ((double) sub1.width() * sub1.height());
        var p2 = Core.countNonZero(sub2) / ((double) sub2.width() * sub2.height());
        var isSub1 = p1 > .4;
        var isSub2 = p2 > (Robot.isBlue ? .2 : .4);

        sub1.release();
        sub2.release();
        img.release();

//        if (isSub1)
            Imgproc.rectangle(_img, bounding1, new Scalar(0, 0, 0));

//        if (isSub2)
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