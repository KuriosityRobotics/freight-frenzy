package com.kuriosityrobotics.firstforward.robot.sensors;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

public class Odometry {
    private final Robot robot;

    private final DcMotor yLeftEncoder;
    private final DcMotor yRightEncoder;
    private final DcMotor mecanumEncoder;

    public Odometry(Robot robot) {
        this.robot = robot;

        yLeftEncoder = robot.getHardware("fLeft");
        yRightEncoder = robot.getHardware("fRight");
        mecanumEncoder = robot.getHardware("bLeft");
    }

    void process() {
        int yLeftEncoderPosition = yLeftEncoder.getCurrentPosition();
        int yRightEncoderPosition = yRightEncoder.getCurrentPosition();
        int mecanumEncoderPosition = mecanumEncoder.getCurrentPosition();
    }

}
