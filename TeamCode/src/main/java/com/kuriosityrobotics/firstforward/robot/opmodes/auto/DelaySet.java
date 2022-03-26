package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DelaySet extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double delay = AutoHelper.delay;
        while ((!isStarted() && !isStopRequested()) || opModeIsActive()) {
            delay = (long) Math.max(0, delay + gamepad1.left_stick_y);

            telemetry.addLine("Delay: " + delay);
            telemetry.update();
        }

        AutoHelper.delay = delay;
    }
}
