package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.HALF_FIELD_MM;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.Units.MM_PER_INCH;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.angleWrap;
import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.doublesEqual;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Objects;

public class VuMarkDetection {
    private final VuforiaTrackable detectedTrackable;
    private final OpenGLMatrix detectedData;
    private final double detectedHorizPeripheralAngle;
    private final double detectedVertPeripheralAngle;
    private final long detectedTime;


    public VuMarkDetection(VuforiaTrackable detectedTrackable, OpenGLMatrix detectedData, double detectedHorizPeripheralAngle, double detectedVertPeripheralAngle, long detectedTime) {
        this.detectedTrackable = detectedTrackable;
        this.detectedData = detectedData;
        this.detectedHorizPeripheralAngle = detectedHorizPeripheralAngle;
        this.detectedVertPeripheralAngle = detectedVertPeripheralAngle;
        this.detectedTime = detectedTime;
    }

    public double getDetectedHorizPeripheralAngle() {
        return detectedHorizPeripheralAngle;
    }

    public double getDetectedVertPeripheralAngle() {
        return detectedVertPeripheralAngle;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        VuMarkDetection that = (VuMarkDetection) o;
        return doublesEqual(that.getDetectedHorizPeripheralAngle(), detectedHorizPeripheralAngle)
                && doublesEqual(that.getDetectedVertPeripheralAngle(), detectedVertPeripheralAngle)
                && detectedTime == that.getDetectedTime() &&
                detectedTrackable.equals(that.getDetectedTrackable()) &&
                detectedData.equals(that.getDetectedData());
    }

    private OpenGLMatrix getDetectedData() {
        return detectedData;
    }

    private VuforiaTrackable getDetectedTrackable() {
        return detectedTrackable;
    }

    public long getDetectedTime() {
        return detectedTime;
    }

    @Override
    public int hashCode() {
        return Objects.hash(detectedTrackable, detectedData, detectedHorizPeripheralAngle, detectedVertPeripheralAngle, detectedTime);
    }

    public Pose getKuriosityPose() {
        var translation = detectedData.getTranslation();
        Point robotLocation = new Point(translation.get(0) / MM_PER_INCH, translation.get(1) / MM_PER_INCH);
        double heading = Orientation.getOrientation(detectedData, EXTRINSIC, XYZ, RADIANS).thirdAngle;

        // Convert from FTC coordinate system to ours
        double robotHeadingOurs = angleWrap(PI - heading);
        double robotXOurs = robotLocation.y + (HALF_FIELD_MM / MM_PER_INCH);
        double robotYOurs = -robotLocation.x + (HALF_FIELD_MM / MM_PER_INCH);

        return new Pose(robotXOurs, robotYOurs, robotHeadingOurs);
    }
}
