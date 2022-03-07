package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoPaths {
    public static final double INTAKE_VELO = 10;
    public static final long VUF_DELAY = 150;

    public static void intakePath(Robot robot, Pose end, long killswitchMillis) {
        PurePursuit path = new PurePursuit(new WayPoint[]{
                new WayPoint(robot.getPose(), INTAKE_VELO),
                new WayPoint(end, new VelocityLock(0))
        }, 4);

        robot.intakeModule.intakePower = 1;

        long start = SystemClock.elapsedRealtime();

        while (robot.running() && !path.atEnd(robot)) {
            if (robot.intakeModule.newMineral || SystemClock.elapsedRealtime() - start >= killswitchMillis) {
                robot.intakeModule.newMineral = false;
                robot.intakeModule.targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
                break;
            } else {
                path.update(robot, robot.drivetrain);
            }
        }
        robot.drivetrain.setMovements(0, 0, 0);
        robot.intakeModule.intakePower = 0;
    }

    public static void waitForVuforia(Robot robot, LinearOpMode linearOpMode, long killSwitch, Pose offsetPoseByIfKilled) {
        long startTime = SystemClock.elapsedRealtime();
        Long sawTime = null;

        while (robot.running()) {
            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - startTime >= killSwitch) {
                robot.sensorThread.resetPose(robot.getPose().add(offsetPoseByIfKilled));
                return;
            }

            if (sawTime == null) {
                long lastAccepted = robot.visionThread.vuforiaLocalizationConsumer.getLastAcceptedTime();
                if (lastAccepted > startTime) {
                    sawTime = lastAccepted;
                }
            } else {
                if (currentTime >= sawTime + VUF_DELAY) {
                    return;
                }
            }

            linearOpMode.sleep(50);
        }
    }

    public static void wallRidePath(Robot robot, PurePursuit path) {
        path.reset();
        while (robot.running()) {
            boolean callAgain = path.update(robot, robot.drivetrain);

            Pose currPose = robot.sensorThread.getPose();

            if (Math.abs(currPose.y - 48) < 1) {
                robot.sensorThread.resetPose(new Pose(currPose.x, currPose.y, Math.toRadians(180)));
            }

            if (!callAgain) {
                return;
            }
        }
    }
}