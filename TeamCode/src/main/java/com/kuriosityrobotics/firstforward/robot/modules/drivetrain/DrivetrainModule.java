package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static com.kuriosityrobotics.firstforward.robot.util.math.MathUtil.max;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.kuriosityrobotics.firstforward.robot.util.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


class DrivetrainModule implements Module, Telemeter {
    //states
    private double xMov, yMov, turnMov;

    //motors
    private final DcMotor fLeft;
    private final DcMotor fRight;
    private final DcMotor bLeft;
    private final DcMotor bRight;

    public DrivetrainModule(HardwareMap hardwareMap) {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        robot.telemetryDump.registerTelemeter(this);
    }

    //updates motor power
    public void update() {
        double fLPower = yMov + turnMov + xMov;
        double fRPower = yMov - turnMov - xMov;
        double bLPower = yMov + turnMov - xMov;
        double bRPower = yMov - turnMov + xMov;

        double scale = scaleDown(fLPower, fRPower, bLPower, bRPower);

        fLPower *= scale;
        fRPower *= scale;
        bLPower *= scale;
        bRPower *= scale;

        setMotorPowers(fLPower, fRPower, bLPower, bRPower);

        long currentTime = SystemClock.elapsedRealtime();
    }

    @Override
    public void onClose() {
        setMotorPowers(0, 0, 0, 0);
    }

    //scale down motor power so largest/smallest is 1/-1
    public double scaleDown(double a, double b, double c, double d) {
        double max = max(a, b, c, d);
        if (max < 1) {
            return 1;
        }
        return max;
    }

    void setMovements(double xMov, double yMov, double turnMov) {
        this.xMov = xMov;
        this.yMov = yMov;
        this.turnMov = turnMov;
    }

    public void setMovements(Pose movementPose) {
        this.setMovements(movementPose.x, movementPose.y, movementPose.heading);
    }

    private void setMotorPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        setMotorPower(fLeft, fLPower);
        setMotorPower(fRight, fRPower);
        setMotorPower(bLeft, bLPower);
        setMotorPower(bRight, bRPower);
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.06) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "DrivetrainModule";
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("xMov: " + xMov);
        data.add("yMov: " + yMov);
        data.add("turnMov: " + turnMov);

        return data;
    }

    @Override
    public int getShowIndex() {
        return 1;
    }
}
