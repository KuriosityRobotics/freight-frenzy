package com.kuriosityrobotics.firstforward.robot.modules.carousel;

import static java.lang.Math.PI;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.modules.Module;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class CarouselModule implements Module, Telemeter {
    private static final double CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE = 2 * PI;
    private static final double CAROUSEL_WHEEL_CIRCUMFERENCE = 15 * PI;
    private static final double MAX_SPEED_MS = 1300;

    //states
    private volatile boolean spin = false;
    private volatile boolean clockwise = false;
    private volatile double maxSpeed;

    private Long spinStartTimeMillis = null;

    //motors
    private final DcMotorEx carouselMotor;

    public CarouselModule(HardwareMap hardwareMap) {
        // original:  1.4 * pi
        this.setMaxSpeed(0.85 * PI);

        carouselMotor = (DcMotorEx) hardwareMap.dcMotor.get("carousel");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public CarouselAction carouselAction() {
        return new CarouselAction(this);
    }

    private double target = 0;

    public void update() {
        this.setClockwise(Robot.isBlue());

        if (isSpin()) {
            if (spinStartTimeMillis == null) {
                spinStartTimeMillis = SystemClock.elapsedRealtime();
            }

            double speed = getMaxSpeed() * Range.clip((((double)(SystemClock.elapsedRealtime() - spinStartTimeMillis)) / MAX_SPEED_MS), 0, 1);
            carouselMotor.setVelocity(isClockwise() ? -speed : speed, AngleUnit.RADIANS);
            target = speed;
        } else {
            spinStartTimeMillis = null;
            carouselMotor.setVelocity(0);
        }
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "CarouselModule";
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("spin: " + isSpin());
        data.add("velocity: " + carouselMotor.getVelocity(AngleUnit.RADIANS));
        data.add("target: " + target);

        return data;
    }

    public boolean isSpin() {
        return spin;
    }

    public void setSpin(boolean spin) {
        this.spin = spin;
    }

    public boolean isClockwise() {
        return clockwise;
    }

    public void setClockwise(boolean clockwise) {
        this.clockwise = clockwise;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    @Override
    public int getShowIndex() {
        return 1;
    }
}
