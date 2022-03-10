package com.kuriosityrobotics.firstforward.robot.opmodes.auto;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.kuriosityrobotics.firstforward.robot.vision.opencv.TeamMarkerDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedCycle extends LinearOpMode {

    public static final Pose RED_START_W = new Pose(9.5, 64.5, Math.toRadians(-90)); //start near warehouse
    public static final Pose FIRST_WOBBLE = new Pose(24, 73, Math.toRadians(-110));

    public static final Pose RED_WOBBLE_W = new Pose(24, 73, Math.toRadians(-110));
    public static final Pose RED_WOBBLE_WALL_POINT = new Pose(7.5, 68, Math.toRadians(180));


    public static final Pose RED_BETWEEN_WOBBLE_WALLGAP = new Pose(7, 62.5, Math.toRadians(180));
    public static final Pose RED_WALL_GAP = new Pose(7, 46.5, Math.toRadians(180));
    private Pose redWarehouse = new Pose(8, 33, Math.toRadians(175));

    public static final Point RED_EXIT_WALLGAP = new Point(9, 64);

    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        robot.resetPose(RED_START_W);
        TeamMarkerDetector.startLocation = TeamMarkerDetector.AutoStartLocation.RED_CYCLE;

        waitForStart();

//        OuttakeModule.VerticalSlideLevel detection = robot.visionThread.getTeamMarkerDetector().getLocation().slideLevel();
        OuttakeModule.VerticalSlideLevel detection = OuttakeModule.VerticalSlideLevel.MID;
        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        PurePursuit redStartwToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(RED_START_W, robot.outtakeModule.extendOuttakeAction(detection)),
                new WayPoint(RED_START_W.between(RED_WOBBLE_W)),
                new WayPoint(FIRST_WOBBLE, 0, wobbleActions)
        }, 4);

        PurePursuit wobbleToWarehouse = new PurePursuit(new WayPoint[]{
                new WayPoint(RED_WOBBLE_W, new VelocityLock(15, false)),
                new WayPoint(RED_BETWEEN_WOBBLE_WALLGAP, new VelocityLock(21
                        , true), robot.intakeModule.intakePowerAction(1)),//, 0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_WALL_GAP),//, 0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(redWarehouse, AutoPaths.INTAKE_VELO)
        }, 4);

        PurePursuit wobbleToWarehouseOdometryOnly = new PurePursuit(new WayPoint[]{
                new WayPoint(RED_WOBBLE_W, new VelocityLock(25, true)),
                new WayPoint(RED_WOBBLE_WALL_POINT, new VelocityLock(25, true)),
                new WayPoint(RED_BETWEEN_WOBBLE_WALLGAP, new VelocityLock(22, true), robot.intakeModule.intakePowerAction(1)),//, 0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_WALL_GAP),//, 0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(redWarehouse, AutoPaths.INTAKE_VELO)
        }, 4);

        ArrayList<Action> exitActions = new ArrayList<>();
        exitActions.add(robot.outtakeModule.extendOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP));
        exitActions.add(robot.intakeModule.intakePowerAction(0));
        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(redWarehouse),
                new WayPoint(RED_WALL_GAP),//,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_EXIT_WALLGAP, exitActions),//,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_WOBBLE_W, 0, robot.outtakeModule.dumpOuttakeAction())
        }, true, 4);

        robot.followPath(redStartwToWobble);

        long startSleep = SystemClock.elapsedRealtime();
//        AutoPaths.waitForVuforia(robot, this, 1250, new Pose(0.25, 0, 0));

        if (detection == OuttakeModule.VerticalSlideLevel.DOWN_NO_EXTEND) {
            sleep(500);
        } else {
            sleep(500);
        }
        assert robot.visionThread.vuforiaLocalizationConsumer != null;
        boolean sawFirst = robot.visionThread.vuforiaLocalizationConsumer.getLastAcceptedTime() >= startSleep;

        int numCycles = 4;
        for (int i = 0; i < numCycles; i++) {
            if (sawFirst) {
                robot.followPath(wobbleToWarehouse);
            } else {
                AutoPaths.wallRidePath(robot, wobbleToWarehouseOdometryOnly);
            }

            Pose intakeVary;

            intakeVary = new Pose(1.5*i, -4, Math.toRadians(-18));

            AutoPaths.intakePath(robot, redWarehouse.add(intakeVary), 4500);

//            if (redWarehouse.y > 7.5)
            if (i % 2 == 1) {
                redWarehouse = redWarehouse.add(new Pose(0, -2, 0));
            }

            if (sawFirst) {
                PurePursuit backToWobble = new PurePursuit(new WayPoint[]{
                        new WayPoint(robot.getPose()),
                        new WayPoint(RED_WALL_GAP, new VelocityLock(42, false)),//,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                        new WayPoint(RED_EXIT_WALLGAP, exitActions),//,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                        new WayPoint(RED_EXIT_WALLGAP.x+7,RED_EXIT_WALLGAP.y+2),
                        new WayPoint(RED_WOBBLE_W, 0, robot.outtakeModule.dumpOuttakeAction())
                }, true, 4);

                robot.followPath(backToWobble);
            } else {
                PurePursuit backToWobble = new PurePursuit(new WayPoint[]{
                        new WayPoint(robot.getPose()),
                        new WayPoint(RED_WALL_GAP.add(new Pose(-1.5, 0, 0)),  new VelocityLock(40, true)),//,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                        new WayPoint(RED_EXIT_WALLGAP.add(new Pose(-1.5, 0, 0)), exitActions),//,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                        new WayPoint(RED_EXIT_WALLGAP.x+7,RED_EXIT_WALLGAP.y+2),
                        new WayPoint(RED_WOBBLE_W.add(new Pose(-1.5, 0, 0)), 0, robot.outtakeModule.dumpOuttakeAction())
                }, true, 4);

                AutoPaths.wallRidePath(robot, backToWobble);
            }

            if (sawFirst) {
                AutoPaths.waitForVuforia(robot, this, 250, new Pose(0, 0, 0));
            } else {
                sleep(150);
            }
        }

        robot.followPath(wobbleToWarehouse);

        /*
        blueStartwToWobble.follow(false);

        for (int i = 0; i < 3; i++){
            robot.followPath(blueWobbleToWarehouse);
            robot.followPath(blueWarehouseToWobble);
        }
         */

    }
}