package com.kuriosityrobotics.firstforward.robot.opmodes.tests.TestAutos;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.modules.outtake.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;
import com.kuriosityrobotics.firstforward.robot.pathfollow.PurePursuit;
import com.kuriosityrobotics.firstforward.robot.pathfollow.VelocityLock;
import com.kuriosityrobotics.firstforward.robot.pathfollow.WayPoint;
import com.kuriosityrobotics.firstforward.robot.util.math.Point;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedCycle extends LinearOpMode {

    public static final Pose RED_START_W = new Pose(7, 64.5, Math.toRadians(90)); //start near warehouse
    public static final Pose RED_WOBBLE_W = new Pose(22, 73, Math.toRadians(-110));
    public static final Pose RED_BETWEEN_WOBBLE_WALLGAP = new Pose(7, 62.5, Math.toRadians(180));
    public static final Point RED_EXIT_WALLGAP = new Point(9, 60);
    public static final Pose RED_WALL_GAP = new Pose(6, 46.5, Math.toRadians(180));
    private Pose redWarehouse = new Pose(8, 34, Math.toRadians(175));

    public void runOpMode() {
        Robot robot = null;
        try {
            robot = new Robot(hardwareMap, telemetry, this);
            robot.resetPose(RED_START_W);
        } catch (Exception e) {
            this.stop();
            e.printStackTrace();
            return;
        }

        ArrayList<Action> wobbleActions = new ArrayList<>();
        wobbleActions.add(robot.outtakeModule.dumpOuttakeAction());
        PurePursuit redStartwToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(RED_START_W),
                new WayPoint(RED_START_W.between(RED_WOBBLE_W), robot.outtakeModule.extendOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP)),
                new WayPoint(RED_WOBBLE_W, 0, wobbleActions)
        }, 4);

        PurePursuit wobbleToWarehouse = new PurePursuit(new WayPoint[]{
                new WayPoint(RED_WOBBLE_W, new VelocityLock(10, false)),
                new WayPoint(RED_BETWEEN_WOBBLE_WALLGAP, new VelocityLock(20, true)),//, 0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_WALL_GAP, robot.intakeModule.intakeAction()),//, 0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(redWarehouse, 0)
        }, 4);

        ArrayList<Action> exitActions = new ArrayList<>();
        exitActions.add(robot.outtakeModule.extendOuttakeAction(OuttakeModule.VerticalSlideLevel.TOP));
        exitActions.add(robot.intakeModule.stopIntakeAction());
        PurePursuit warehouseToWobble = new PurePursuit(new WayPoint[]{
                new WayPoint(redWarehouse),
                new WayPoint(RED_WALL_GAP),//,  0.7 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_EXIT_WALLGAP, exitActions),//,  0.55 * MotionProfile.ROBOT_MAX_VEL, new ArrayList<>()),
                new WayPoint(RED_WOBBLE_W, 0, robot.outtakeModule.dumpOuttakeAction())
        }, true, 4);

        waitForStart();

        robot.followPath(redStartwToWobble);

        for (int i = 0; i < 3; i++) {
            wobbleToWarehouse.reset();
            robot.followPath(wobbleToWarehouse);

            redWarehouse = redWarehouse.add(new Pose(0, -5, 0));

            warehouseToWobble.reset();
            robot.followPath(warehouseToWobble);
        }

        wobbleToWarehouse.reset();
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
