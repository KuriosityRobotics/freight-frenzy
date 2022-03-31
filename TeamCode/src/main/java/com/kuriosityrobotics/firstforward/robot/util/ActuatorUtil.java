package com.kuriosityrobotics.firstforward.robot.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ActuatorUtil {
    public static void setRunMode(DcMotor.RunMode mode, DcMotor... motors) {
        assert motors.length > 0;

        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public static void setMotorPower(double power, DcMotor... motors) {
        assert motors.length > 0;

        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public static void setTarget(int target, DcMotor... motors) {
        assert motors.length > 0;

        for (DcMotor motor : motors) {
            motor.setTargetPosition(target);
        }
    }
}
