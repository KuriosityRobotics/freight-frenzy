package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.intake.IntakeModule;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.kuriosityrobotics.firstforward.robot.vision.vuforia.VuforiaLocalizationConsumer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoPaths {
    public static final double INTAKE_VELO = 10;
    public static final long VUF_DELAY = 150;

    public static double delay = 0;
    public static OuttakeModule.VerticalSlideLevel delayedStartLogic(LinearOpMode opMode, Robot robot, Pose reset) {
        while (!robot.started() && robot.running()) {
            delay = Math.max(0, delay - (opMode.gamepad1.left_stick_y * 0.01));
            robot.getTelemetryDump().setAlert("Auto start delay (ms): " + delay);
        }

        robot.resetPose(reset);
        OuttakeModule.VerticalSlideLevel detected = AutoPaths.awaitBarcodeDetection(robot);

        robot.getTelemetryDump().setAlert("Currently delaying for " + delay + " milliseconds.");
        opMode.sleep((long) delay);

        robot.getTelemetryDump().clearAlert();

        return detected;
    }

    public static OuttakeModule.VerticalSlideLevel awaitBarcodeDetection(Robot robot) {
        VuforiaLocalizationConsumer vufConsumer = robot.getVisionThread().getVuforiaLocalizationConsumer();

        vufConsumer.setManualCamHeading(Math.PI);

        TeamMarkerDetector.TeamMarkerLocation location;
        robot.getVisionThread().getTeamMarkerDetector().activate();
        do {
            if (!robot.running())
                return null;

            location = robot.getVisionThread().getTeamMarkerDetector().getLocation();

            Log.v("AUTO", "barcoding");
        } while ((location == null || location == TeamMarkerDetector.TeamMarkerLocation.UNKNOWN));

        robot.getVisionThread().getTeamMarkerDetector().deactivate();

        vufConsumer.disableManualCam();

        return location.slideLevel();
    }

    /**
     * THIS METHOD MANUALLY SETS THE CAMERA POSITION TO MATH.PI AND RETURNS. THIS MAY CAUSE THE
     * CAMERA TO STOP TRACKING IF NOBODY UNSETS THAT LATER!
     *
     * @param robot
     */
    public static void calibrateVuforia(Robot robot) {
        while (robot.running() && !robot.getVisionThread().isStarted()) {
            // wait for vuforia to start
        }

        VuforiaLocalizationConsumer vufConsumer = robot.getVisionThread().getVuforiaLocalizationConsumer();

        vufConsumer.setManualCamHeading(0);

        Pose expectedRobotPosition = new Pose(29.375, 64.5, Math.toRadians(90));
        if (Robot.isBlue()) {
            expectedRobotPosition = expectedRobotPosition.fieldMirror();
        }

        Pose gottenPosition;
        do {
            if (!robot.running())
                return;

            gottenPosition = vufConsumer.getLastVuforiaPosition();
        } while (gottenPosition == null);

        Log.v("VUF", "gotten: " + gottenPosition);

        Pose offsetBy = expectedRobotPosition.minus(gottenPosition);

        Log.v("VUF", "offsetby: " + offsetBy);

        vufConsumer.changeAllianceWallOffsetBy(offsetBy);

        vufConsumer.setManualCamHeading(Math.PI);
    }

    public static void intakePath(Robot robot, Pose end, long killswitchMillis) {
        Pose complete = end.add(new Pose(0, -36, 0));
        if (complete.y < 10) {
            complete = new Pose(end.x, 10, end.heading);
        }

        PurePursuit path = new PurePursuit(new WayPoint[]{
                new WayPoint(robot.getPose(), new VelocityLock(INTAKE_VELO, false)),
                new WayPoint(end),
                new WayPoint(complete, new VelocityLock(0))
        }, 4);

        robot.getIntakeModule().intakePower = 1;
        robot.getIntakeModule().newMineral = false;

        long start = SystemClock.elapsedRealtime();

        while (robot.running() && !path.atEnd(robot)) {
            if (robot.getIntakeModule().hasMineral() || robot.getIntakeModule().newMineral) {
                robot.getIntakeModule().targetIntakePosition = IntakeModule.IntakePosition.RETRACTED;
                robot.getIntakeModule().intakePower = 0;

                robot.getIntakeModule().newMineral = false;
                break;
            } else if (SystemClock.elapsedRealtime() - start >= killswitchMillis) {
                break;
            } else {
                path.update(robot, robot.getDrivetrain());
            }
        }
        robot.getDrivetrain().setMovements(0, 0, 0);
    }

    public static void waitForVuforia(Robot robot, LinearOpMode linearOpMode, long killSwitch, Pose offsetPoseByIfKilled) {
        long startTime = SystemClock.elapsedRealtime();
        Long sawTime = null;

        while (robot.running()) {
            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime - startTime >= killSwitch) {
                robot.getSensorThread().resetPose(robot.getPose().add(offsetPoseByIfKilled));
                return;
            }

            if (sawTime == null) {
                long lastAccepted = robot.getVisionThread().getVuforiaLocalizationConsumer().getLastAcceptedTime();
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
            boolean callAgain = path.update(robot, robot.getDrivetrain());

            Pose currPose = robot.getSensorThread().getPose();

            if (Math.abs(currPose.y - 49) < 1) {
                robot.getSensorThread().resetPose(Robot.isBlue() ? Pose.fieldMirror(7.25, currPose.y, Math.toRadians(180)) : new Pose(7.25, currPose.y, Math.toRadians(180)));
            }

            if (!callAgain) {
                return;
            }
        }
    }
}