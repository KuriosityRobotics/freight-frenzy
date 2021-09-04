package com.kuriosityrobotics.firstforward.robot.modules;
import com.kuriosityrobotics.firstforward.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;


public class DrivetrainModule implements Module{
    private final Robot robot;
    private final Boolean isOn = true;

    //states
    public double xMov = 0;
    public double yMov = 0;
    public double turnMov = 0;

    //motors
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    public DrivetrainModule(Robot robot) {
        this.robot = robot;
        initModules();
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
    }

    //scale down motor power so largest/smallest is 1/-1
    public double scaleDown(double a, double b, double c, double d){
        double max = Math.max(Math.abs(d), Math.max(Math.abs(c), Math.max(Math.abs(a), Math.abs(b))));
        if (max < 1) {return 1;}
        return max;
    }

    public void setMovements(double xMov, double yMov, double turnMov){
        this.xMov = xMov;
        this.yMov = yMov;
        this.turnMov = turnMov;
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

    //@Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("forwards movement: " + yMov);
        data.add("strafe movement: " + xMov);
        data.add("turn movement: " + turnMov);
        return data;
    }

    public void initModules() {
        fLeft = robot.getDcMotor("fLeft");
        fRight = robot.getDcMotor("fRight");
        bLeft = robot.getDcMotor("bLeft");
        bRight = robot.getDcMotor("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isOn() { return isOn; }

    public String getName() {
        return "DriveTrainModule";
    }
}
