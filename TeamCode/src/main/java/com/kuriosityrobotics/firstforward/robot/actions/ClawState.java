package com.kuriosityrobotics.firstforward.robot.actions;

import com.kuriosityrobotics.configuration.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import java.util.Optional;

// example
public class ClawState implements DesiredState {
    @Config(configName = "claw.distance")
    public static long CLAW_DISTANCE;

    // note that we are not muckiung arond with Robot;  ClawState's dependencies are passed to it in the constructor
    final DcMotor jointOneMotor;
    final DcMotor jointTwoMotor;

    // empty until first execution
    private long lastExecutionTime = -1;

    public ClawState(DcMotor jointOneMotor, DcMotor jointTwoMotor) {
        this.jointOneMotor = jointOneMotor;
        this.jointTwoMotor = jointTwoMotor;
    }


    public Optional<Long> getLastExecutionTime() {
        if(this.lastExecutionTime == -1)
            return Optional.empty();
        else
            return Optional.of(this.lastExecutionTime);
    }

    public void tick(Optional<Long> timeSinceLastExecution) {
        lastExecutionTime = System.nanoTime();
        jointOneMotor.setPower(59.);
        jointTwoMotor.setPower(.5);

    }

    public boolean achieved() {
        return true /*jointOneMotor.getMotorPosition() > 1*/;
    }
}
