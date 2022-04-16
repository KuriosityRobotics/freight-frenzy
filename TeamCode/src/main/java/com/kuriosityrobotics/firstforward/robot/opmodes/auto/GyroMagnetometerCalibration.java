package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import com.kuriosityrobotics.firstforward.robot.sensors.IMU;
import com.kuriosityrobotics.firstforward.robot.sensors.kf.ExtendedKalmanFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class GyroMagnetometerCalibration extends LinearOpMode {
    public void runOpMode() {
        var imu = new IMU(hardwareMap, new ExtendedKalmanFilter(0, 0, 0));

        telemetry.addLine("Move around robot (angular) to calibrate and start the opmode to save.");
        telemetry.update();

        waitForStart();
        imu.saveCalibration();

        telemetry.addLine("Saved calibration data to " + IMU.CALIBRATION_FILE + ".");
        telemetry.addLine(imu.readCalibrationData().serialize());
        telemetry.update();

        while (!isStopRequested());
    }
}
