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
    public static final double INTAKE_VELO = 15;
    public static final long VUF_DELAY = 150;

    public static void intakePath(Robot robot, Pose end, long killswitchMillis) {
        Pose complete = end.add(new Pose(0, -10, 0));
        if (complete.y < 10) {
            complete = new Pose(end.x, 10, end.heading);
        }

        PurePursuit path = new PurePursuit(new WayPoint[]{
                new WayPoint(robot.getPose(), new VelocityLock(INTAKE_VELO, false)),
                new WayPoint(end),
                new WayPoint(complete, new VelocityLock(0))
        }, 4);

        robot.intakeModule.intakePower = 1;
        robot.intakeModule.newMineral = false;

        long start = SystemClock.elapsedRealtime();

        while (robot.running() && !path.atEnd(robot)) {
            if (robot.intakeModule.hasMineral() || robot.intakeModule.newMineral) {
                robot.intakeModule.targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
                robot.intakeModule.intakePower = 0;
                Log.v("auto", "LEAVING!: " + robot.intakeModule.newMineral + " time?? " + (SystemClock.elapsedRealtime() - start >= killswitchMillis));

                robot.intakeModule.newMineral = false;
                break;
            } else if (SystemClock.elapsedRealtime() - start >= killswitchMillis) {
                break;
            } else {
                path.update(robot, robot.drivetrain);
            }
        }
        Log.v("auto", "DONE!");
        robot.drivetrain.setMovements(0, 0, 0);
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

            if (Math.abs(currPose.y - 49) < 1) {
                robot.sensorThread.resetPose(Robot.isBlue ? Pose.fieldMirror(7.25, currPose.y, Math.toRadians(180)) : new Pose(7.25, currPose.y, Math.toRadians(180)));
            }

            if (!callAgain) {
                return;
            }
        }
    }

    public static void calibrateVuforia(Robot robot) {
        while (robot.running() && !robot.visionThread.started) {
            // wait for vuforia to start
        }

        Pose expectedRobotPosition = new Pose(29.375, 64.5, Math.toRadians(90));
        if (Robot.isBlue) {
            expectedRobotPosition = expectedRobotPosition.fieldMirror();
        }

        Pose gottenPosition = null;
        do {
            gottenPosition = robot.visionThread.vuforiaLocalizationConsumer.lastRawRobotPosition();
        } while (gottenPosition == null && robot.running());

        Log.v("VUF", "gotten: " + gottenPosition);

        Pose offsetBy = expectedRobotPosition.minus(gottenPosition);

        Log.v("VUF", "offsetby: " + offsetBy);

        robot.visionThread.vuforiaLocalizationConsumer.offsetAllianceWallBy(offsetBy);

        robot.visionThread.vuforiaLocalizationConsumer.doneCalibrating = true;
    }
}