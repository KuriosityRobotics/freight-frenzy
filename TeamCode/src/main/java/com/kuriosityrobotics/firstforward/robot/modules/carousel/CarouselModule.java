package com.kuriosityrobotics.firstforward.robot.modules.carousel;

import static java.lang.Math.PI;

import android.os.SystemClock;
import android.util.Log;

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
//    private static final double REVS_PER_CAROUSEL_REV = CAROUSEL_WHEEL_CIRCUMFERENCE / CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE;
    private static final double MAX_SPEED_MS = 1300;

    //states
    public volatile boolean spin = false;
    public volatile boolean clockwise = false;
    public volatile double maxSpeed;
    public volatile double startPosition;

    private Long spinStartTimeNanos = null;

    //motors
    private final DcMotorEx carouselMotor;

    public CarouselModule(HardwareMap hardwareMap) {
        // original:  1.4 * pi
        this.setMaxSpeed(1.2 * PI);

        carouselMotor = (DcMotorEx) hardwareMap.dcMotor.get("carousel");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public CarouselAction carouselAction() {
        return new CarouselAction(this);
    }

    private double target = 0;

    private double speed;

    public void update() {
//        Log.v("carousel", "currentpos: " + carouselMotor.getCurrentPosition());
        this.setClockwise(Robot.isBlue());

        if (isSpin()) {
            if (spinStartTimeNanos == null) {
                spinStartTimeNanos = SystemClock.elapsedRealtimeNanos();
                startPosition = carouselMotor.getCurrentPosition();
            }

            speed = getMaxSpeed() * Range.clip((((double)(SystemClock.elapsedRealtimeNanos() - spinStartTimeNanos)) / MAX_SPEED_MS), 0, 1);

            // 1035 ticks in 360 degrees
            if ((startPosition - carouselMotor.getCurrentPosition()) > 520) {
                Log.v("carousel",  "max speed");
                speed = maxSpeed;
            } else Log.v("carousel", "no max speed");

            carouselMotor.setVelocity(clockwise ? -speed : speed, AngleUnit.RADIANS);
            target = speed;
        } else {
            spinStartTimeNanos = null;
            carouselMotor.setVelocity(0);
        }
    }

    public boolean isOn() {
        return true;
    }

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

    public boolean isSpin() {return spin;}

    public void setSpin(boolean spin) {
        this.spin = spin;
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
    public int getShowIndex() {return 1;}
}
