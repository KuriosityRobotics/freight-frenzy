package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D;
import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.geometry.euclidean.twod.AffineTransformMatrix2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static java.lang.Math.*;
import static org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D.*;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;

import java.util.ArrayList;
import java.util.List;

public class PinholeCamera {


    private static final Vector3D robotToCameraTranslationBase =
            Vector3D.of(0.3, 16.404, 4.821);
    private static final AffineTransformMatrix3D robotToCameraRotation = AffineTransformMatrix3D.createRotation(Vector3D.ZERO
            , QuaternionRotation.fromAxisAngle(Vector3D.of(1, 0, 0), PI / 6));
    private static final double SENSOR_DIAGONAL = 6 * 0.0393700787;
    private static final double FRAME_WIDTH = 800;
    private static final double FOCAL_LENGTH_X = 578.272;
    private static final double O_X = 402.145;

    private static final double FRAME_HEIGHT = 448;
    private static final double O_Y = 221.506;
    // in pixels
    private double focalLength, originX, originY, width, height;
    // in units (diagonal;  used with hypot(width, height) to convert between pixels and coords)
    private final double sensorDiagonalUnits;

    private PinholeCamera(double focalLength, double originX, double originY, double width, double height,
                          double sensorDiagonalUnits) {
        this.originX = originX;
        this.originY = originY;
        this.width = width;
        this.height = height;
        this.sensorDiagonalUnits = sensorDiagonalUnits;
        this.focalLength = focalLength;
    }

    public static PinholeCamera create() {
        return new PinholeCamera(
                FOCAL_LENGTH_X,
                O_X,
                O_Y,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                SENSOR_DIAGONAL);
    }

    public ArrayList<Vector2D> getCubePixelCoords(Mat original){
        ArrayList<Vector2D> cubePixel = new ArrayList<>();
        Mat img = original.clone();
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGBA2RGB);

        Mat canny = new Mat();
        cubeCanny(img, canny);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double points;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect bound = Imgproc.boundingRect(contour);
            points = (bound.width + 1) * (bound.height + 1);

            if (points < (FRAME_WIDTH * FRAME_HEIGHT)/200 || points > (FRAME_WIDTH * FRAME_HEIGHT)/10 || /*contourArea(contour)/bound.area() < 0.3 ||*/
                    (double) Math.max(bound.width, bound.height)/Math.min(bound.width, bound.height) > 100 || bound.height < FRAME_HEIGHT/40 || bound.height > FRAME_HEIGHT/1.5 || bound.width < FRAME_WIDTH/40 || bound.width > FRAME_WIDTH/1.5){
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

        return cubePixel;
    }

    public ArrayList<Vector2D> getBallPixelCoords(Mat original){
        ArrayList<Vector2D> ballPixel = new ArrayList<>();
        Mat img = original.clone();
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGBA2RGB);

        Mat canny = new Mat();
        ballCanny(img, canny);

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

    public static void cubeCanny(Mat original, Mat canny){
        Mat img = original.clone();

        //cropColors(img);
        filterCubeColors(img);
        //HighGui.imshow("before canny", img);

        Imgproc.morphologyEx(img, img, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.morphologyEx(img, img, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.blur(img, img, new Size(3, 3));

        Imgproc.Canny(img, canny, 50, 100);
    }
    public static void ballCanny(Mat original, Mat canny){
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
    public static void filterCubeColors(Mat img){
        double blue, red, green;

        for (int row = 0; row < img.rows(); row++){
            for (int col = 0; col < img.cols(); col++){
                blue = img.get(row, col)[2];
                green = img.get(row, col)[1];
                red = img.get(row, col)[0];

                if (blue < 0.6 * green && green < 1.1 * red && green > 0.25 * red && red > 20 && green > 20){
                    img.put(row, col, blue, green ,red);
                }else{
                    img.put(row, col,50, 50, 50);
                }
            }
        }
    }
    public static void filterBallColors(Mat img){
        double blue, red, green;

        for (int row = 0; row < img.rows(); row++){
            for (int col = 0; col < img.cols(); col++){
                blue = img.get(row, col)[2];
                green = img.get(row, col)[1];
                red = img.get(row, col)[0];

                if (blue > 100 && green > 100 && red > 100 && isBallScalar(new Scalar(blue, green, red))){
                    img.put(row, col, blue, green, red);
                }else{
                    img.put(row, col, 0, 0, 0);
                }
            }
        }
    }
    public static boolean isCubeScalar(Scalar s){
        double red = s.val[0];
        double green = s.val[1];
        double blue = s.val[2];

        //return blue < 0.55 * green && green < 1.1 * red && green > 0.35 * red;
        return red > 25 && green > 25 && green > 0.3 * red && green < 1.5 * red && blue < 0.5 * red;
    }

    public static boolean isBallScalar(Scalar s){
        double red = s.val[0];
        double green = s.val[1];
        double blue = s.val[2];;

        double avg = (red + green + blue)/3;
        double deviation = Math.abs(red - avg)/avg + Math.abs(green - avg)/avg + Math.abs(blue - avg)/avg;

        return deviation < 0.3 && red > 160 && green > 160 && blue > 167;
    }
    private static double getZPrimeConstrainedColumn(AffineTransformMatrix3D worldToFrame, int column, double constraint, double x, double y) {
        var frameToWorld = worldToFrame.inverse().toArray();
        int start = column * 4;
        double e = frameToWorld[start], f = frameToWorld[start + 1], g = frameToWorld[start + 2], h = frameToWorld[start + 3];
        return (constraint - h) / (e * x + f * y + g);
    }

    private static double getZPrimeConstrainedX(AffineTransformMatrix3D worldToFrame, double X, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 0, X, x, y);
    }

    private static double getZPrimeConstrainedY(AffineTransformMatrix3D worldToFrame, double Y, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 1, Y, x, y);
    }

    private static double getZPrimeConstrainedZ(AffineTransformMatrix3D worldToFrame, double Z, double x, double y) {
        return getZPrimeConstrainedColumn(worldToFrame, 1, Z, x, y);
    }

    private AffineTransformMatrix2D normaliseFrameCoordinates() { // TODO:  stop abusing the word 'normalise'
        return AffineTransformMatrix2D.createScale(1, -1).translate(-originX, originY);
    }

    AffineTransformMatrix3D cameraMatrix(Vector3D robotPosition, double cameraAngle, double globalHeading) {
        var variableRotation = Vector3D.of(2.5f * sin(cameraAngle), 0, 2.5f * cos(cameraAngle)); // <0, -3.365>
        var fieldToRobotRotation = createRotation(Vector3D.ZERO, QuaternionRotation.fromAxisAngle(Vector3D.of(0, 1, 0), (globalHeading)));

        var translation = robotToCameraTranslationBase.add(variableRotation);
        translation = fieldToRobotRotation.apply(translation);
        return (robotToCameraRotation).multiply(fieldToRobotRotation)
                .premultiply(createTranslation(robotPosition.add(translation).negate()))
//                .premultiply()
                .premultiply(createScale(Math.hypot(width, height) / sensorDiagonalUnits))
                .premultiply(createScale(focalLength, focalLength, 1));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, double u, double v) {
        var normalisedCoordinates = Vector2D.of(u, v).transform(normaliseFrameCoordinates());
        double x = normalisedCoordinates.getX(), y = normalisedCoordinates.getY();

        double zPrime = getZPrimeConstrainedY(cameraMatrix(robotPosition, cameraAngle, globalHeading), 0, x, y);
        double xPrime = x * zPrime, yPrime = y * zPrime;

        return cameraMatrix(robotPosition, cameraAngle, globalHeading).inverse().apply(Vector3D.of(xPrime, yPrime, zPrime));
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, double u, double v, double freightHeight) {
        var normalisedCoordinates = Vector2D.of(u, v).transform(normaliseFrameCoordinates());
        double x = normalisedCoordinates.getX(), y = normalisedCoordinates.getY();

        double zPrime = getZPrimeConstrainedY(cameraMatrix(robotPosition, cameraAngle, globalHeading), freightHeight, x, y);
        double xPrime = x * zPrime, yPrime = y * zPrime;

        return cameraMatrix(robotPosition, cameraAngle, globalHeading).inverse().apply(Vector3D.of(xPrime, yPrime, zPrime));
    }


    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, Vector2D locationOnFrame) {
        return getLocationOnField(robotPosition, cameraAngle, globalHeading, locationOnFrame.getX(), locationOnFrame.getY());
    }

    public Vector3D getLocationOnField(Vector3D robotPosition, double cameraAngle, double globalHeading, Vector2D locationOnFrame, double freightHeight) {
        return getLocationOnField(robotPosition, cameraAngle, globalHeading, locationOnFrame.getX(), locationOnFrame.getY(), freightHeight);
    }

    public Vector2D getLocationOnFrame(Vector3D robotPosition, double cameraAngle, double globalHeading, Vector3D position) {
        var transformed = position.transform(cameraMatrix(robotPosition, cameraAngle, globalHeading));

        var res = transformed.multiply(1 / transformed.getZ());
        return Vector2D.of(res.getX(), res.getY()).transform(normaliseFrameCoordinates().inverse());
    }

    public final PositionBoundPinholeCamera bindToPose(Vector3D robotPosition, double cameraAngle, double globalHeading) {
        return this.new PositionBoundPinholeCamera(robotPosition, cameraAngle, globalHeading);
    }

    public class PositionBoundPinholeCamera {
        public final Vector3D robotPosition;
        public final double cameraAngle, globalHeading;

        PositionBoundPinholeCamera(Vector3D robotPosition, double cameraAngle, double globalHeading) {
            this.robotPosition = robotPosition;
            this.cameraAngle = cameraAngle;
            this.globalHeading = globalHeading;
        }

        public Vector2D getLocationOnFrame(Vector3D position) {
            return PinholeCamera.this.getLocationOnFrame(robotPosition, cameraAngle, globalHeading, position);
        }

        public Vector3D getLocationOnField(double u, double v) {
            return PinholeCamera.this.getLocationOnField(robotPosition, cameraAngle, globalHeading, u, v);
        }

        public Vector3D getLocationOnField(Vector2D point) {
            return PinholeCamera.this.getLocationOnField(robotPosition, cameraAngle, globalHeading, point);
        }

    }
}
