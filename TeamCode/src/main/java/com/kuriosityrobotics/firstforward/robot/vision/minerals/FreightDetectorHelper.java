package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/*
    Note 1: Canny is an ALGORITHM.
    Note 2:
 */

public class FreightDetectorHelper {
    public FreightDetectorHelper(PinholeCamera camera) { }

    public enum FreightType{
        CUBE,
        BALL,
    }

    public ArrayList<Vector2D> getCargoPixels(Mat original) {
        Log.v("pinhole", "START cube coords");
        ArrayList<Vector2D> cubePixel = new ArrayList<>();
        Mat img = original.clone();
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGBA2RGB);

        Mat canny = new Mat();
        cubeContourDetection(img, canny);
        Log.v("pinhole", "CANNY cube coords");

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double points;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect bound = Imgproc.boundingRect(contour);
            points = (bound.width + 1) * (bound.height + 1);

            if (points < (PinholeCamera.FRAME_WIDTH * PinholeCamera.FRAME_HEIGHT)/200 || points > (PinholeCamera.FRAME_WIDTH * PinholeCamera.FRAME_HEIGHT)/10 || /*contourArea(contour)/bound.area() < 0.3 ||*/
                    (double) Math.max(bound.width, bound.height)/Math.min(bound.width, bound.height) > 100 || bound.height < PinholeCamera.FRAME_HEIGHT/40 || bound.height > PinholeCamera.FRAME_HEIGHT/1.5 || bound.width < PinholeCamera.FRAME_WIDTH/40 || bound.width > PinholeCamera.FRAME_WIDTH/1.5){
                continue;
            }

            //scale down bound by 3
            org.opencv.core.Point topLeft = new org.opencv.core.Point(bound.x + bound.width/2. - bound.width/6., bound.y + bound.height/2. - bound.height/6.);
            Rect smallBound = new Rect(topLeft, new Size(bound.width/3., bound.height/3.));
            Scalar mean = Core.mean(original.submat(smallBound));
            if (isCubeScalar(mean)){
                cubePixel.add(Vector2D.of(bound.x + bound.width/2., bound.y + bound.height));
            }
        }
        Log.v("pinhole", "FILTER cube coords");

        return cubePixel;
    }

    public ArrayList<Vector2D> getBallPixelCoords(Mat original) {
        ArrayList<Vector2D> ballPixel = new ArrayList<>();
        Mat img = original.clone();
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGBA2RGB);

        Mat canny = new Mat();
        ballContourDetection(img, canny);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double points;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect bound = Imgproc.boundingRect(contour);
            points = (bound.width + 1) * (bound.height + 1);

            if (points < (img.width() * img.height())/300. || points > (img.width() * img.height())/10. || /*contourArea(contour)/bound.area() < 0.3 ||*/
                    (double) Math.max(bound.width, bound.height)/Math.min(bound.width, bound.height) > 1.5 || bound.height < img.height()/40 || bound.height > img.height()/1.5 || bound.width < img.width()/40 || bound.width > img.width()/1.5){
                continue;
            }

            //scale down bound by 3
            org.opencv.core.Point topLeft = new org.opencv.core.Point(bound.x + bound.width/2. - bound.width/6., bound.y + bound.height/2. - bound.height/2.5);
            Rect smallBound = new Rect(topLeft, new Size(bound.width/3., bound.height/3.));
            Scalar mean = Core.mean(original.submat(smallBound));
            if (isBallScalar(mean)){
                ballPixel.add(Vector2D.of(bound.x + bound.width/2., bound.y + bound.height/2.));
            }
        }

        return ballPixel;
    }

    public static void cubeContourDetection(Mat original, Mat canny) {
        Mat img = original.clone();

        //cropColors(img);
        filterCubeColors(img);
        //HighGui.imshow("before canny", img);

        Imgproc.morphologyEx(img, img, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.morphologyEx(img, img, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.blur(img, img, new Size(3, 3));

        Imgproc.Canny(img, canny, 50, 100);
    }

    public static void ballContourDetection(Mat original, Mat canny) {
        Mat img = original.clone();
        //Imgproc.morphologyEx(img, img, MORPH_CLOSE, Imgproc.getStructuringElement(MORPH_ELLIPSE, new Size(7, 7)));
        //Imgproc.morphologyEx(img, img, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));
        //Imgproc.blur(img, img, new Size(3, 3));
        Imgproc.morphologyEx(img, img, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9)));

        filterBallColors(img);
        Imgproc.GaussianBlur(img, img, new Size(5,5),0,0);
        //HighGui.imshow("before canny", img);

        Imgproc.Canny(img, canny, 60, 180);
    }

    public static void filterCubeColors(Mat img) {
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2HLS);

        Mat mask = new Mat();
        Core.inRange(img, new Scalar(14, 0, 0), new Scalar(30, 255, 255), mask);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_HLS2RGB);
        Mat result = new Mat();
        img.copyTo(result, mask);
        result.copyTo(img);
    }

    public static void filterBallColors(Mat img) {
        double blue, red, green;

        for (int row = 0; row < img.rows(); row++) {
            for (int col = 0; col < img.cols(); col++) {
                blue = img.get(row, col)[2];
                green = img.get(row, col)[1];
                red = img.get(row, col)[0];

                if (blue > 100 && green > 100 && red > 100 && isBallScalar(new Scalar(blue, green, red))) {
                    img.put(row, col, blue, green, red);
                } else {
                    img.put(row, col, 0, 0, 0);
                }
            }
        }
    }

    public static boolean isCubeScalar(Scalar s) {
        double red = s.val[0];
        double green = s.val[1];
        double blue = s.val[2];

        //return blue < 0.55 * green && green < 1.1 * red && green > 0.35 * red;
        return red > 25 && green > 25 && green > 0.3 * red && green < 1.5 * red && blue < 0.5 * red;
    }

    public static boolean isBallScalar(Scalar s) {
        double red = s.val[0];
        double green = s.val[1];
        double blue = s.val[2];
        //Core.meanStdDev();
        double avg = (red + green + blue)/3;
        double deviation = Math.abs(red - avg)/avg + Math.abs(green - avg)/avg + Math.abs(blue - avg)/avg;

        return deviation < 0.3 && red > 160 && green > 160 && blue > 167;
    }

    public Point findFreightPos(Vector2D pixelCoords, double cameraHeading, Pose robotPose, FreightType freightType) {
        Point filmCoords = new Point(PinholeCamera.INCHES_PER_PIXEL * (pixelCoords.getX() - PinholeCamera.O_X), -PinholeCamera.INCHES_PER_PIXEL * (pixelCoords.getY() - PinholeCamera.O_Y));
        double adjustedBaseHeading = -robotPose.getHeading() + Math.PI/2;
        double[] camBase = new double[]{robotPose.getX() + 4.821 * Math.cos(adjustedBaseHeading) + 0.318 * Math.sin(adjustedBaseHeading),
                robotPose.getY() + 4.821 * Math.sin(adjustedBaseHeading) - 0.318 * Math.cos(adjustedBaseHeading),
                16.404};

        double adjustedPosHeading = -(cameraHeading + robotPose.getHeading()) + Math.PI/2;
        double[] camPos = new double[]{camBase[0] + 3.365 * Math.cos(adjustedPosHeading), camBase[1] + 3.365 * Math.sin(adjustedPosHeading), 16.404};

        //in terms of actual xy plane
        double finalYaw = -(cameraHeading + robotPose.getHeading() + Math.atan2(filmCoords.getX(), PinholeCamera.FOCAL_LENGTH)) + Math.PI/2;
        double finalPitch = Math.toRadians(-30) + Math.atan2(filmCoords.getY(), PinholeCamera.FOCAL_LENGTH);

        if (finalPitch >= 0) {
            return new Point(9999, 9999);
        }
        //System.out.println(Math.toDegrees(finalPitch));
        double t;
        if (freightType == FreightType.CUBE) {
            t = (0 - camPos[2]) / Math.sin(finalPitch);
        } else{
            t = (1.375 - camPos[2]) / Math.sin(finalPitch);
        }
        //System.out.println(t);
        double finalX = camPos[0] + t * Math.cos(finalYaw) * Math.cos(finalPitch);
        double finalY = camPos[1] + t * Math.sin(finalYaw) * Math.cos(finalPitch);
        //TODO
        return new Point(finalX, finalY);
    }
}
