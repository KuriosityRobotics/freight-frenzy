package com.kuriosityrobotics.firstforward.robot.opmodes.tests;

import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.pathfollow.actions.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Straight extends LinearOpMode {
    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
        }

        PurePursuit pp = new PurePursuit(robot, new WayPoint[]{
                new WayPoint(0, 0, 0, new Action() {
                    @Override
                    public void tick() {
                        super.tick();
                        Log.v("action", "yeet");
                    }

                    @Override
                    public boolean isCompleted() {
                        return true;
                    }
                }),
                new WayPoint(0, 20),
                new WayPoint(0, 40, new Action() {
//                new WayPoint(-30, 40, new Action() {
                    @Override
                    public void tick() {
                        super.tick();
                        Log.v("action", "yoot");
                    }

                    @Override
                    public boolean isCompleted() {
                        return true;
                    }
                }),
//                new WayPoint(-30, 60, 0)
                new WayPoint(0, 50, 0)
        }, 10);

        waitForStart();

        pp.follow();

        while (opModeIsActive()) {
            // yeet
        }
    }
}
