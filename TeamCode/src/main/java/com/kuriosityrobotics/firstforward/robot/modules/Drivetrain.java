package com.kuriosityrobotics.firstforward.robot.modules;

import com.kuriosityrobotics.firstforward.robot.Robot;

public class Drivetrain extends ModuleCollection{
    Robot robot;
    public boolean isOn;

    //modules
    private DrivetrainModule drivetrainModule;

    //states
    double xMov = 0;
    double yMov = 0;
    double turnMov = 0;

    public void update(){
        if (drivetrainModule.isOn()) {
            applyMovements();
            drivetrainModule.update();
        }
    }

    public void setMovements(double xMov, double yMov, double turnMov){
        this.xMov = xMov;
        this.yMov = yMov;
        this.turnMov = turnMov;
    }

    public void applyMovements(){
        drivetrainModule.setMovements(xMov, yMov, turnMov);
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
