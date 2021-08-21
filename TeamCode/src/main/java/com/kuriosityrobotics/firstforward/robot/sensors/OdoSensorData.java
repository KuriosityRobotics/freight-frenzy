package com.kuriosityrobotics.firstforward.robot.sensors;

import java.util.Objects;


//java moment
public class OdoSensorData {
    private final int yLeftEncoderPosition;
    private final int yRightEncoderPosition;
    private final int mecanumEncoderPosition;

    public OdoSensorData(int yLeftEncoderPosition, int yRightEncoderPosition, int mecanumEncoderPosition) {
        this.yLeftEncoderPosition = yLeftEncoderPosition;
        this.yRightEncoderPosition = yRightEncoderPosition;
        this.mecanumEncoderPosition = mecanumEncoderPosition;
    }

    public int getyLeftEncoderPosition() {
        return yLeftEncoderPosition;
    }

    public int getyRightEncoderPosition() {
        return yRightEncoderPosition;
    }

    public int getMecanumEncoderPosition() {
        return mecanumEncoderPosition;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        OdoSensorData that = (OdoSensorData) o;
        return getyLeftEncoderPosition() == that.getyLeftEncoderPosition() && getyRightEncoderPosition() == that.getyRightEncoderPosition() && getMecanumEncoderPosition() == that.getMecanumEncoderPosition();
    }

    @Override
    public int hashCode() {
        return Objects.hash(getyLeftEncoderPosition(), getyRightEncoderPosition(), getMecanumEncoderPosition());
    }

    @Override
    public String toString() {
        return "OdoSensorData{" +
                "yLeftEncoderPosition=" + yLeftEncoderPosition +
                ", yRightEncoderPosition=" + yRightEncoderPosition +
                ", mecanumEncoderPosition=" + mecanumEncoderPosition +
                '}';
    }


}
