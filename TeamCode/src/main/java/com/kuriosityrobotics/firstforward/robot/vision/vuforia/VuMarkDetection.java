package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import com.kuriosityrobotics.firstforward.robot.util.math.Point;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Objects;

import lombok.Data;

@Data
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

    public VuforiaTrackable getDetectedTrackable() {
        return detectedTrackable;
    }

    public OpenGLMatrix getDetectedData() {
        return detectedData;
    }

    public double getDetectedHorizPeripheralAngle() {
        return detectedHorizPeripheralAngle;
    }

    public double getDetectedVertPeripheralAngle() {
        return detectedVertPeripheralAngle;
    }

    public long getDetectedTime() {
        return detectedTime;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        VuMarkDetection that = (VuMarkDetection) o;
        return Double.compare(that.detectedHorizPeripheralAngle, detectedHorizPeripheralAngle) == 0 && Double.compare(that.detectedVertPeripheralAngle, detectedVertPeripheralAngle) == 0 && detectedTime == that.detectedTime && detectedTrackable.equals(that.detectedTrackable) && detectedData.equals(that.detectedData);
    }

    @Override
    public int hashCode() {
        return Objects.hash(detectedTrackable, detectedData, detectedHorizPeripheralAngle, detectedVertPeripheralAngle, detectedTime);
    }
}
