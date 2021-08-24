package com.kuriosityrobotics.firstforward.robot.sensors;


import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OdoSensors {
    private final DcMotor yLeftEncoder;
    private final DcMotor yRightEncoder;
    private final DcMotor mecanumEncoder;

    private volatile int yLeftEncoderPosition = -1;
    private volatile int yRightEncoderPosition = -1;
    private volatile int mecanumEncoderPosition = -1;

    public OdoSensors(Robot robot) {
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
