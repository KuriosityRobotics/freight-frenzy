package com.kuriosityrobotics.firstforward.robot.opmodes;

import android.util.Log;
import static com.kuriosityrobotics.firstforward.robot.math.MathUtil.angleWrap;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.math.Pose;
import com.kuriosityrobotics.firstforward.robot.modules.OuttakeModule;
import com.kuriosityrobotics.firstforward.robot.util.Button;
import com.kuriosityrobotics.firstforward.robot.util.ClassicalPID;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.kuriosityrobotics.firstforward.robot.util.Constants.OpModes.JOYSTICK_EPSILON;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot = null;

    Button retractButton = new Button();

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap, telemetry, this, Autonomous.PARK);
        } catch (Exception e) {
            this.stop();
            throw new RuntimeException(e);
        }
        robot.visionThread.managedCamera.activateCamera(robot.frontCamera);
        waitForStart();

        while (opModeIsActive()) {
            // yeet
            updateDrivetrainStates();
            //robot.drivetrain.setBrakePose(new Pose(0,24,0));
            updateIntakeStates();
            updateOuttakeStates();
            updateCarouselStates();
            //robot.drivetrain.setBrakePose(new Pose(10,0,0));
            updateCameraStates();
        }
    }

    ClassicalPID angularWallrideController = new ClassicalPID(0.5, 0, 1);
    ClassicalPID distanceWallrideController = new ClassicalPID(0.03, 0, 0.7);
    private void updateDrivetrainStates() {
        double yMov = Math.signum(gamepad1.left_stick_y) * -Math.pow(gamepad1.left_stick_y, 2);
        double xMov = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double turnMov = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            yMov /= 2;
            xMov /= 2;
            turnMov /= 2;
        }

        robot.drivetrain.setMovements(xMov, yMov, turnMov);

        Pose robotPose = robot.sensorThread.getPose();
        //if it's in the area for wall riding
        if (robotPose.isInRange(0, 28, 15, 68)){
            Log.i("TeleOp", "NEAR GsdadsAP");
            //angle reference: 0 is down, 90 is left 180/-180 is up
            //angle should be 0 or 180 or -180, finds closest one
            double targetHeading = closestNum(robot.sensorThread.getPose().heading, new double[]{-Math.PI, 0, Math.PI});
            double targetX = 6.5;

            //Log.i("TeleOp", "distance from x = 6.5: "  + (targetX - robotPose.getX()));

            double angleFactor = angularWallrideController.calculateSpeed(targetHeading - robotPose.heading);
            double distanceFactor = distanceWallrideController.calculateSpeed(targetX - robotPose.getX());

            //Log.i("TeleOp", "distanceFactor: "  + distanceFactor);

            turnMov += angleFactor;
            xMov += 1/0.9 * distanceFactor * Math.sin(robotPose.heading);
            yMov += distanceFactor * Math.cos(robotPose.heading);
            Log.i("TeleOp", "xMov add: " + xMov);
            Log.i("TeleOp", "yMov add: " + yMov);
        }else{
            angularWallrideController.reset();
            distanceWallrideController.reset();
        }

        robot.drivetrain.setMovements(xMov, yMov, turnMov);
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = Math.abs(gamepad2.left_stick_y) > JOYSTICK_EPSILON
                ? Math.signum(gamepad2.left_stick_y)
                : 0;

        if (retractButton.isSelected(gamepad2.a)) {
            robot.intakeModule.retractIntake = true;
        }
    }

    private void updateOuttakeStates() {
        if (gamepad2.dpad_down)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.DOWN;
        if (gamepad2.dpad_up)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.TOP;
        if (gamepad2.dpad_right)
            OuttakeModule.slideLevel = OuttakeModule.VerticalSlideLevel.MID;

        if (gamepad2.left_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_INWARDS);
        else if (gamepad2.right_bumper)
            robot.outtakeModule.dump(OuttakeModule.HopperDumpPosition.DUMP_OUTWARDS);

        if (gamepad2.right_stick_x == 0 && gamepad2.right_stick_y == 0) {
            OuttakeModule.pivotHeading = 0;
        } else{
            OuttakeModule.pivotHeading = angleWrap(Math.atan2(gamepad2.right_stick_x, gamepad2.right_stick_y) - Math.PI);
        }
        OuttakeModule.skipRotate = gamepad2.y;
    }

    private void updateCarouselStates() {
        robot.carouselModule.spin = gamepad2.x;
    }

    private void updateCameraStates() {
        WebcamName targetCam = robot.frontCamera;
//        if (robot.sensorThread.getPose().heading < (3 * Math.PI / 4) && robot.sensorThread.getPose().heading > (-Math.PI / 4))
//            targetCam = robot.leftCamera;
//        else
//            targetCam = robot.frontCamera;

        robot.visionThread.activeCamera = targetCam;
    }
}
