package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

public class Odometry {
    private final Robot robot;
    private final OdoSensors sensors;

    public Odometry(Robot robot) {
        this.robot = robot;
        this.sensors = new OdoSensors();
    }

    void process() {
        var data = sensors.get();
    }

    //java moment
    private static class OdoSensorData {
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

    private class OdoSensors {
        private final DcMotor yLeftEncoder;
        private final DcMotor yRightEncoder;
        private final DcMotor mecanumEncoder;

        private volatile int yLeftEncoderPosition = -1;
        private volatile int yRightEncoderPosition = -1;
        private volatile int mecanumEncoderPosition = -1;

        public OdoSensors() {
            yLeftEncoder = robot.getHardware("fLeft");
            yRightEncoder = robot.getHardware("fRight");
            mecanumEncoder = robot.getHardware("bLeft");
        }

        OdoSensorData get() {
            yLeftEncoderPosition = yLeftEncoder.getCurrentPosition();
            yRightEncoderPosition = yRightEncoder.getCurrentPosition();
            mecanumEncoderPosition = mecanumEncoder.getCurrentPosition();

            return new OdoSensorData(yLeftEncoderPosition, yRightEncoderPosition, mecanumEncoderPosition);
        }

        public int getYLeftEncoderPosition() {
            return yLeftEncoderPosition;
        }

        public int getYRightEncoderPosition() {
            return yRightEncoderPosition;
        }

        public int getMecanumEncoderPosition() {
            return mecanumEncoderPosition;
        }
    }


}
