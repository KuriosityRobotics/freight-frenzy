package com.kuriosityrobotics.firstforward.robot.modules;

import static java.lang.Math.PI;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class CarouselModule implements Module, Telemeter {
    private static final double CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE = 2 * PI;
    private static final double CAROUSEL_WHEEL_CIRCUMFERENCE = 15 * PI;
    private static final double REVS_PER_CAROUSEL_REV = CAROUSEL_WHEEL_CIRCUMFERENCE / CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE;

    private final Robot robot;
    private final boolean isOn = true;

    //states
    public volatile boolean spin = false;

    private Long spinStartTimeMillis = null;

    //motors
    private DcMotorEx carouselMotor;

    public CarouselModule(Robot robot) {
        this.robot = robot;
        carouselMotor = (DcMotorEx) robot.getDcMotor("carouselTurner");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static final double MAX_CAROUSEL_SPEED = Math.toRadians(250);
    private static final double MAX_SPEED_MS = 1600;

    public void update() {
        if (spin) {
            if (spinStartTimeMillis == null) {
                spinStartTimeMillis = SystemClock.elapsedRealtime();
            }

            double speed = MAX_CAROUSEL_SPEED * Range.clip(((SystemClock.elapsedRealtime() - spinStartTimeMillis) / MAX_SPEED_MS), 0, 1);
            carouselMotor.setVelocity(-speed, AngleUnit.RADIANS);
        } else {
            spinStartTimeMillis = null;
            carouselMotor.setVelocity(0);
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public String getName() {
        return "CarouselModule";
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("spin: " + spin);

        return data;
    }
}
