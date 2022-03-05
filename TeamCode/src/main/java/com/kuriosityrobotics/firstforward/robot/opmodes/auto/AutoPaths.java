package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;

public class AutoPaths {
    public static final double INTAKE_VELO = 20;

    public static void intakePath(Robot robot, Pose end, long killswitchMillis) {
        PurePursuit path = new PurePursuit(new WayPoint[]{
                new WayPoint(robot.getPose(), INTAKE_VELO),
                new WayPoint(end, new VelocityLock(0))
        }, 4);

        robot.intakeModule.intakePower = 1;

        long start = SystemClock.elapsedRealtime();

        while (robot.running() && !path.atEnd(robot)) {
            if (robot.intakeModule.hasMineral() || SystemClock.elapsedRealtime() - start >= killswitchMillis) {
                robot.intakeModule.intakePower = 0;
                robot.drivetrain.setMovements(0, 0, 0);
                return;
            } else {
                path.update(robot, robot.drivetrain);
            }
        }
    }
}
