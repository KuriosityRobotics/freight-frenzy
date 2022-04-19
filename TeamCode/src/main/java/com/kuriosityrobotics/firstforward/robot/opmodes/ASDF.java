package com.kuriosityrobotics.firstforward.robot.opmodes;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.util.wrappers.AnalogDistance;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class ASDF extends LinearOpMode {
    private AnalogDistance distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        this.distanceSensor = new AnalogDistance(hardwareMap.get(AnalogInput.class, "distance"));

        while (opModeIsActive()) {
            Log.v("IN", "read: " + distanceSensor.getSensorReading());
        }
    }
}
